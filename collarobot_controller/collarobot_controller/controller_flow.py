import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
import sys
import select
from pathlib import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from collarobot_msgs.action import MoveIngredient, Gesture
from enum import Enum
import json
from typing import Dict, Optional


# Import the model models to get the main_brain feedback
import collarobot_controller.models as models

DATA_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_controller' 
    / 'data'
)

class States(Enum):
    IDLE = 0
    WAIT_UNTIL_HUMAN_PUT_INGREDIENT = 1
    MAIN_BRAIN = 2
    PROPOSE_NEW_INGREDIENT = 3
    PUT_INGREDIENT_BACK = 4
    ACCEPT_INGREDIENT = 5
    HAND_INVITATION = 6
    HAND_CIRCLE = 7
    WAIT_AFTER_CIRCLE = 8
    END = 9


class VisionNodeSubscriber(Node):
    def __init__(self):
        super().__init__('collarobot_vision_node_subscriber')
        self.received_msg = None
        self.subscription = self.create_subscription(
            String, '/collarobot/state', self.listener_callback, 10)
        self.latest_data = None
        self.previous_data = None  # Store previous state for comparison
        self.new_ingredient_flag = False
        # Ingredient IDs moved by the robot — ignore these in change detection
        # so that vision reflecting the robot's own moves is never treated as
        # a new human action, regardless of how late the update arrives.
        self._robot_moved_ids: set = set()

    def listener_callback(self, msg):
        self.latest_data = msg.data

        if self.is_ingredient_status_changed():
            self.get_logger().info('New numbers added to proposed or accepted zones!')
            self.new_ingredient_flag = True

    def check_and_reset_new_ingredient_flag(self) -> bool:
        flag = self.new_ingredient_flag
        self.new_ingredient_flag = False
        return flag

    def add_robot_moved_id(self, ingredient_id: int):
        """Register an ingredient ID as robot-moved.

        Vision updates showing this ID appearing in proposed/accepted will be
        ignored until the next genuine human action clears it.
        """
        self._robot_moved_ids.add(ingredient_id)

    def is_ingredient_status_changed(self):
        """
        Checks if any ingredient changed zone (storage/proposed/accepted).
        Detects additions, removals, and transfers between zones.
        Ignores changes caused by the robot (tracked via _robot_moved_ids).
        Returns True if a genuine human change is detected, False otherwise.
        """
        if not self.latest_data:
            return False

        try:
            current_data = json.loads(self.latest_data)

            if self.previous_data is None:
                self.previous_data = current_data
                return False

            # Build ingredient -> zone mappings for previous and current state
            prev_zones = self._build_zone_map(self.previous_data)
            curr_zones = self._build_zone_map(current_data)

            # Update previous data for next comparison
            self.previous_data = current_data

            # Find IDs whose zone changed
            all_ids = set(prev_zones.keys()) | set(curr_zones.keys())
            changed_ids = set()
            for ing_id in all_ids:
                if prev_zones.get(ing_id) != curr_zones.get(ing_id):
                    changed_ids.add(ing_id)

            # Filter out robot-moved IDs
            if self._robot_moved_ids:
                robot_ids = changed_ids & self._robot_moved_ids
                if robot_ids:
                    self.get_logger().info(
                        f'Ignoring robot-moved IDs in vision: {robot_ids}'
                    )
                    self._robot_moved_ids -= robot_ids  # consumed
                changed_ids -= robot_ids

            if changed_ids:
                details = {
                    i: f'{prev_zones.get(i, "?")} -> {curr_zones.get(i, "?")}'
                    for i in changed_ids
                }
                self.get_logger().info(f'Zone changes detected: {details}')
                return True

            return False

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error checking status change: {e}')
            return False

    @staticmethod
    def _build_zone_map(data: dict) -> Dict[int, str]:
        """Build {ingredient_id: zone_name} from a vision state dict."""
        zone_map = {}
        for zone in ('storage', 'proposed', 'accepted'):
            for ing_id in data.get(zone, []):
                zone_map[int(ing_id)] = zone
        return zone_map



class MainStateMachineNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.current_state = States.IDLE
        self.next_state = States.IDLE
        self.cycle_done = True
        self.wait_start_time = 0.0
        self.last_tick_time = 0.0
        self.vision_subscriber = VisionNodeSubscriber()

        _ing_path = DATA_PATH / 'ingredients.json'
        with open(_ing_path) as f:
            self._name_to_id: Dict[str, int] = json.load(f)

        cb = ReentrantCallbackGroup()
        self._move_client = ActionClient(
            self, MoveIngredient, '/collarobot/move_ingredient', callback_group=cb,
        )
        self._gesture_client = ActionClient(
            self, Gesture, '/collarobot/gesture', callback_group=cb,
        )
        self._robot_busy = False
        self._gesture_busy = False
        self._pending_ingredient_id: Optional[int] = None
        self._pending_ingredient_name: Optional[str] = None
        self._skip_after_action: bool = False
        self.rejected: set = set()   # ingredient names rejected this session
        self.excluded: set = set()   # recipe names to exclude from prediction

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Node started with models.py integration.")

    def subscribe_from_vision(self):
        return self.vision_subscriber.check_and_reset_new_ingredient_flag()

    def send_gesture_robot_controller(self, gesture_name: str):
        self.get_logger().info(f'>>> SENDING GESTURE: {gesture_name}')
        goal = Gesture.Goal()
        goal.gesture_name = gesture_name
        goal.return_position = 'home'
        self._gesture_busy = True
        self._gesture_client.send_goal_async(goal).add_done_callback(
            self._gesture_goal_response_cb
        )

    def _gesture_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Gesture goal REJECTED by gesture_node')
            self._gesture_busy = False
            return
        handle.get_result_async().add_done_callback(self._gesture_result_cb)

    def _gesture_result_cb(self, future):
        result = future.result().result
        self._gesture_busy = False
        # Clear any stale flag that accumulated during the gesture
        self.vision_subscriber.check_and_reset_new_ingredient_flag()
        if result.success:
            self.get_logger().info('Gesture done')
        else:
            self.get_logger().error(f'Gesture FAILED: {result.message}')


    def send_motion_robot_controller(self, ingredient_id: Optional[int], destination: str):
        if ingredient_id is None:
            self.get_logger().warn('send_motion_robot_controller: no ingredient ID set')
            return
        self.get_logger().info(f'>>> SENDING MOTION: ingredient {ingredient_id} -> {destination}')
        # Tell vision to ignore this ingredient's zone change — it's the robot acting
        self.vision_subscriber.add_robot_moved_id(ingredient_id)
        goal = MoveIngredient.Goal()
        goal.ingredient_id = ingredient_id
        goal.destination = destination
        self._robot_busy = True
        self._move_client.send_goal_async(goal).add_done_callback(
            self._move_goal_response_cb
        )

    def _move_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Move goal REJECTED by motion coordinator')
            self._robot_busy = False
            return
        handle.get_result_async().add_done_callback(self._move_result_cb)

    def _move_result_cb(self, future):
        result = future.result().result
        self._robot_busy = False
        # Clear any stale flag that accumulated during the move
        self.vision_subscriber.check_and_reset_new_ingredient_flag()
        if result.success:
            self.get_logger().info(
                f'Move done: {result.pick_position} -> {result.place_position}'
            )
            if self._skip_after_action:
                self.get_logger().info('Lookahead triggered -> transitioning to HAND_CIRCLE')
                self._skip_after_action = False
                self.next_state = States.HAND_CIRCLE
                self.current_state = States.HAND_CIRCLE
        else:
            self.get_logger().error(f'Move FAILED: {result.message}')

    def timer_callback(self):
        match self.current_state:
            case States.IDLE:
                if self.cycle_done:
                    self.cycle_done = False
                    self.rejected = set()
                    self.excluded = set()
                    self.next_state = States.HAND_INVITATION

            case States.HAND_INVITATION:
                self.send_gesture_robot_controller("wiggle")
                if self.subscribe_from_vision():
                    self.next_state = States.MAIN_BRAIN
                else:
                    self.next_state = States.WAIT_UNTIL_HUMAN_PUT_INGREDIENT

            case States.WAIT_UNTIL_HUMAN_PUT_INGREDIENT:
                if self._robot_busy:
                    return  # don't react to vision while robot is moving
                if self.subscribe_from_vision():
                    self.next_state = States.MAIN_BRAIN

            case States.MAIN_BRAIN:
                latest = self.vision_subscriber.latest_data
                if latest:
                    data = json.loads(latest)
                    def ids_to_names(id_list):
                        return {n for i in id_list if (n := models.get_ingredient_name(i)) is not None}
                    available = ids_to_names(data.get('storage', []))
                    proposed  = ids_to_names(data.get('proposed', []))
                    accepted  = ids_to_names(data.get('accepted', []))
                else:
                    available = proposed = accepted = set()

                action, ingredient = models.pick_action(
                    accepted, proposed, available,
                    rejected=self.rejected, excluded=self.excluded,
                )
                self.get_logger().info(f"Model Decision: {action} -> {ingredient}")
                self._pending_ingredient_name = ingredient
                self._pending_ingredient_id = self._name_to_id.get(ingredient) if ingredient else None

                # Lookahead: simulate the effect of this action and check
                # whether the model would skip on the next turn.
                self._skip_after_action = False
                if action in ("accepted", "proposed", "rejected"):
                    sim_accepted = set(accepted)
                    sim_proposed = set(proposed)
                    sim_available = set(available)
                    if action == "accepted" and ingredient:
                        sim_accepted.add(ingredient)
                        sim_proposed.discard(ingredient)
                    elif action == "proposed" and ingredient:
                        sim_proposed.add(ingredient)
                        sim_available.discard(ingredient)
                    elif action == "rejected" and ingredient:
                        sim_proposed.discard(ingredient)
                    next_action, _ = models.pick_action(
                        sim_accepted, sim_proposed, sim_available,
                        rejected=self.rejected, excluded=self.excluded,
                    )
                    if next_action == "skip":
                        self.get_logger().info(
                            "Lookahead: model would skip next turn -> HAND_CIRCLE after this move"
                        )
                        self._skip_after_action = True

                if action == "proposed":
                    self.next_state = States.PROPOSE_NEW_INGREDIENT
                elif action == "rejected":
                    self.next_state = States.PUT_INGREDIENT_BACK
                elif action == "accepted":
                    self.next_state = States.ACCEPT_INGREDIENT
                elif action == "skip":
                    self.next_state = States.HAND_CIRCLE
                else:
                    self.get_logger().warn(f"Unknown decision: {action}. Staying in Brain.")

            case States.PROPOSE_NEW_INGREDIENT:
                self.send_motion_robot_controller(self._pending_ingredient_id, 'proposal')
                self.next_state = States.WAIT_UNTIL_HUMAN_PUT_INGREDIENT

            case States.PUT_INGREDIENT_BACK:
                if self._pending_ingredient_name:
                    self.rejected.add(self._pending_ingredient_name)
                self.send_motion_robot_controller(self._pending_ingredient_id, 'storage')
                self.next_state = States.WAIT_UNTIL_HUMAN_PUT_INGREDIENT

            case States.ACCEPT_INGREDIENT:
                self.send_motion_robot_controller(self._pending_ingredient_id, 'accepted')
                self.next_state = States.WAIT_UNTIL_HUMAN_PUT_INGREDIENT

            case States.HAND_CIRCLE:
                self.send_gesture_robot_controller("circle")

                latest = self.vision_subscriber.latest_data
                if latest:
                    data = json.loads(latest)
                    def ids_to_names(id_list):
                        return {n for i in id_list if (n := models.get_ingredient_name(i)) is not None}
                    accepted  = ids_to_names(data.get('accepted', []))
                    proposed  = ids_to_names(data.get('proposed', []))
                    available = ids_to_names(data.get('storage', []))
                    recipe, _ = models.predict_recipe(accepted, self.rejected, available, proposed, self.excluded)
                    self.get_logger().info(f"\n*** Finished recipe: {recipe} ***\nHappy cooking! :)")

                self.wait_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.last_tick_time = self.wait_start_time
                self.get_logger().info("\n--- 20 SECOND WAIT START ---")
                self.next_state = States.WAIT_AFTER_CIRCLE

            case States.WAIT_AFTER_CIRCLE:
                now = self.get_clock().now().seconds_nanoseconds()[0]
                elapsed = now - self.wait_start_time
                remaining = int(20 - elapsed)

                if now - self.last_tick_time >= 1.0:
                    self.get_logger().info(f"Time remaining: {remaining}s...")
                    self.last_tick_time = now

                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    user_input = sys.stdin.readline().strip().lower()
                    if user_input == 'y':
                        self.next_state = States.IDLE
                        self.cycle_done = True

                if elapsed >= 20:
                    self.next_state = States.END

            case States.END:
                self.get_logger().warn("Shutdown.")
                rclpy.shutdown()

        if self.current_state != self.next_state:
            self.get_logger().info(f"TRANSITION: {self.current_state.name} -> {self.next_state.name}")
            self.current_state = self.next_state


def main(args=None):
    rclpy.init(args=args)
    node = MainStateMachineNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.vision_subscriber)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.vision_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()