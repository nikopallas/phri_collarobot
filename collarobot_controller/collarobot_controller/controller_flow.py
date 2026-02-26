import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import sys
import select
import threading
from pathlib import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from collarobot_msgs.action import MoveIngredient, Gesture
from enum import Enum
import json
from typing import Dict, Optional

import collarobot_controller.models as models

DATA_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_controller' / 'data'
)


class States(Enum):
    IDLE = 0
    HAND_INVITATION = 1
    WAIT_FOR_HUMAN = 2
    MAIN_BRAIN = 3
    PROPOSE_NEW_INGREDIENT = 4
    PUT_INGREDIENT_BACK = 5
    ACCEPT_INGREDIENT = 6
    WAIT_FOR_ROBOT = 7
    HAND_CIRCLE = 8
    WAIT_AFTER_CIRCLE = 9
    END = 10


class VisionNodeSubscriber(Node):
    def __init__(self):
        super().__init__('collarobot_vision_node_subscriber')
        self.subscription = self.create_subscription(
            String, '/collarobot/state', self._on_vision, 10)
        self.latest_data = None
        self._previous_data = None
        self._human_acted = threading.Event()
        self._robot_moved_ids: set = set()
        self._robot_moved_lock = threading.Lock()

    def _on_vision(self, msg):
        self.latest_data = msg.data
        if self._detect_human_change():
            self.get_logger().info('Human changed the table')
            self._human_acted.set()

    def consume_human_flag(self) -> bool:
        """Thread-safe: return True if human acted, then reset."""
        if self._human_acted.is_set():
            self._human_acted.clear()
            return True
        return False

    def add_robot_moved_id(self, ingredient_id: int):
        with self._robot_moved_lock:
            self._robot_moved_ids.add(ingredient_id)

    def remove_robot_moved_id(self, ingredient_id: int):
        """Remove an ID from the filter (e.g. after a failed move)."""
        with self._robot_moved_lock:
            self._robot_moved_ids.discard(ingredient_id)

    def _detect_human_change(self) -> bool:
        if not self.latest_data:
            return False
        try:
            current = json.loads(self.latest_data)
            if self._previous_data is None:
                self._previous_data = current
                return False

            prev = self._build_zone_map(self._previous_data)
            curr = self._build_zone_map(current)
            self._previous_data = current

            changed = {i for i in prev.keys() | curr.keys()
                       if prev.get(i) != curr.get(i)}

            # Filter robot's own moves
            with self._robot_moved_lock:
                robot = changed & self._robot_moved_ids
                if robot:
                    self.get_logger().info(
                        f'Ignoring robot-moved IDs: {robot}')
                    self._robot_moved_ids -= robot
                changed -= robot

            if changed:
                detail = {i: f'{prev.get(i,"?")} -> {curr.get(i,"?")}'
                          for i in changed}
                self.get_logger().info(f'Zone changes: {detail}')
                return True
            return False
        except Exception as e:
            self.get_logger().error(f'Vision parse error: {e}')
            return False

    @staticmethod
    def _build_zone_map(data: dict) -> Dict[int, str]:
        zone_map = {}
        for zone in ('storage', 'proposed', 'accepted'):
            for ing_id in data.get(zone, []):
                zone_map[int(ing_id)] = zone
        return zone_map


def _read_vision(vision, logger) -> tuple[set, set, set]:
    """Return (available, proposed, accepted) name sets from latest vision."""
    latest = vision.latest_data
    if not latest:
        return set(), set(), set()
    try:
        data = json.loads(latest)
    except (json.JSONDecodeError, Exception) as e:
        logger.error(f'_read_vision JSON error: {e}')
        return set(), set(), set()
    to_names = lambda ids: {
        n for i in ids if (n := models.get_ingredient_name(i)) is not None
    }
    return (
        to_names(data.get('storage', [])),
        to_names(data.get('proposed', [])),
        to_names(data.get('accepted', [])),
    )


class MainStateMachineNode(Node):
    """
    State machine flow:

        IDLE -> HAND_INVITATION -> WAIT_FOR_HUMAN -> MAIN_BRAIN
                                                        |
                    .-----------------------------------'
                    |  "skip"  -> HAND_CIRCLE -> WAIT_AFTER_CIRCLE -> END
                    |  "proposed" / "accepted" / "rejected"
                    v
              PROPOSE / ACCEPT / REJECT -> WAIT_FOR_ROBOT
                                              |
                    skip lookahead: -----------> HAND_CIRCLE
                    otherwise: ------------------> WAIT_FOR_HUMAN
    """

    def __init__(self):
        super().__init__('controller_node')

        self._state = States.IDLE
        self._next = States.IDLE
        self._cycle_done = True
        self._wait_start = 0.0
        self._last_tick = 0.0
        self.vision = VisionNodeSubscriber()

        with open(DATA_PATH / 'ingredients.json') as f:
            self._name_to_id: Dict[str, int] = json.load(f)

        cb = ReentrantCallbackGroup()
        self._move_client = ActionClient(
            self, MoveIngredient, '/collarobot/move_ingredient',
            callback_group=cb,
        )
        self._gesture_client = ActionClient(
            self, Gesture, '/collarobot/gesture', callback_group=cb,
        )

        # Flags set by async callbacks, consumed by timer
        self._robot_busy = False
        self._gesture_busy = False

        # Decision state
        self._pending_id: Optional[int] = None
        self._pending_name: Optional[str] = None
        self._skip_after_move: bool = False
        self.rejected: set = set()
        self.excluded: set = set()

        self.create_timer(0.1, self._tick)
        self.get_logger().info("Controller started.")

    def _reset_cycle(self):
        """Clear all transient state for a fresh cycle."""
        self.rejected = set()
        self.excluded = set()
        self._skip_after_move = False
        self._pending_id = None
        self._pending_name = None
        self._robot_busy = False
        self._gesture_busy = False
        self.vision.consume_human_flag()  # discard stale flag

    # --- Gesture (fire-and-forget) ----------------------------------

    def _send_gesture(self, name: str):
        self.get_logger().info(f'>>> GESTURE: {name}')
        goal = Gesture.Goal()
        goal.gesture_name = name
        goal.return_position = 'home'
        self._gesture_busy = True
        self._gesture_client.send_goal_async(goal).add_done_callback(
            self._on_gesture_accepted)

    def _on_gesture_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Gesture REJECTED')
            self._gesture_busy = False
            return
        handle.get_result_async().add_done_callback(self._on_gesture_result)

    def _on_gesture_result(self, future):
        self._gesture_busy = False
        if not future.result().result.success:
            self.get_logger().error('Gesture FAILED')

    # --- Move ingredient (result read by timer) ---------------------

    def _send_move(self, ingredient_id: Optional[int], destination: str) -> bool:
        """Send a move goal. Returns False if the move could not be sent."""
        if ingredient_id is None:
            self.get_logger().warn('_send_move: no ingredient ID')
            return False
        self.get_logger().info(
            f'>>> MOVE: ingredient {ingredient_id} -> {destination}')
        self.vision.add_robot_moved_id(ingredient_id)
        goal = MoveIngredient.Goal()
        goal.ingredient_id = ingredient_id
        goal.destination = destination
        self._robot_busy = True
        self._move_client.send_goal_async(goal).add_done_callback(
            self._on_move_accepted)
        return True

    def _on_move_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Move REJECTED')
            self._robot_busy = False
            return
        handle.get_result_async().add_done_callback(self._on_move_result)

    def _on_move_result(self, future):
        """Only set flags — timer owns all state transitions."""
        result = future.result().result
        if not result.success:
            self.get_logger().error(f'Move FAILED: {result.message}')
            # Clean up vision filter — ingredient was never moved
            if self._pending_id is not None:
                self.vision.remove_robot_moved_id(self._pending_id)
        else:
            self.get_logger().info(
                f'Move done: {result.pick_position} -> {result.place_position}')
        self._robot_busy = False

    # --- Timer: single thread, owns all transitions -----------------

    def _tick(self):
        match self._state:

            case States.IDLE:
                if self._cycle_done:
                    self._cycle_done = False
                    self._reset_cycle()
                    self._next = States.HAND_INVITATION

            case States.HAND_INVITATION:
                self._send_gesture("wiggle")
                self._next = States.WAIT_FOR_HUMAN

            case States.WAIT_FOR_HUMAN:
                if self._gesture_busy:
                    return
                if self.vision.consume_human_flag():
                    self._next = States.MAIN_BRAIN

            case States.MAIN_BRAIN:
                available, proposed, accepted = _read_vision(
                    self.vision, self.get_logger())

                # Pass COPIES — pick_action mutates its arguments
                action, ingredient = models.pick_action(
                    set(accepted), set(proposed), set(available),
                    rejected=set(self.rejected),
                    excluded=set(self.excluded),
                )
                self.get_logger().info(f"Decision: {action} -> {ingredient}")

                self._pending_name = ingredient
                self._pending_id = (
                    self._name_to_id.get(ingredient) if ingredient else None
                )

                # Lookahead: if the model would say "skip" after this
                # action, go straight to HAND_CIRCLE when the move finishes
                # instead of waiting for the human.
                self._skip_after_move = False
                if action in ("accepted", "proposed", "rejected"):
                    sim_a = set(accepted)
                    sim_p = set(proposed)
                    sim_av = set(available)
                    sim_rej = set(self.rejected)
                    if action == "accepted" and ingredient:
                        sim_a.add(ingredient)
                        sim_p.discard(ingredient)
                    elif action == "proposed" and ingredient:
                        sim_p.add(ingredient)
                        sim_av.discard(ingredient)
                    elif action == "rejected" and ingredient:
                        sim_p.discard(ingredient)
                        sim_rej.add(ingredient)
                    next_action, _ = models.pick_action(
                        sim_a, sim_p, sim_av,
                        rejected=sim_rej,
                        excluded=set(self.excluded),
                    )
                    if next_action == "skip":
                        self.get_logger().info(
                            "Lookahead: will skip to HAND_CIRCLE after move")
                        self._skip_after_move = True

                match action:
                    case "proposed":
                        self._next = States.PROPOSE_NEW_INGREDIENT
                    case "rejected":
                        self._next = States.PUT_INGREDIENT_BACK
                    case "accepted":
                        self._next = States.ACCEPT_INGREDIENT
                    case "skip":
                        self._next = States.HAND_CIRCLE
                    case _:
                        self.get_logger().warn(f"Unknown action: {action}")

            case States.PROPOSE_NEW_INGREDIENT:
                if self._send_move(self._pending_id, 'proposal'):
                    self._next = States.WAIT_FOR_ROBOT
                else:
                    self._next = States.WAIT_FOR_HUMAN

            case States.PUT_INGREDIENT_BACK:
                if self._pending_name:
                    self.rejected.add(self._pending_name)
                if self._send_move(self._pending_id, 'storage'):
                    self._next = States.WAIT_FOR_ROBOT
                else:
                    self._next = States.WAIT_FOR_HUMAN

            case States.ACCEPT_INGREDIENT:
                if self._send_move(self._pending_id, 'accepted'):
                    self._next = States.WAIT_FOR_ROBOT
                else:
                    self._next = States.WAIT_FOR_HUMAN

            case States.WAIT_FOR_ROBOT:
                if self._robot_busy:
                    return
                # Move finished — check lookahead, then wait for human
                if self._skip_after_move:
                    self._skip_after_move = False
                    self.get_logger().info(
                        'Lookahead triggered -> HAND_CIRCLE')
                    self._next = States.HAND_CIRCLE
                else:
                    self._next = States.WAIT_FOR_HUMAN

            case States.HAND_CIRCLE:
                self._send_gesture("circle")
                available, proposed, accepted = _read_vision(
                    self.vision, self.get_logger())
                if accepted or proposed:
                    recipe, _ = models.predict_recipe(
                        accepted, self.rejected, available,
                        proposed, self.excluded,
                    )
                    self.get_logger().info(
                        f"\n*** Finished recipe: {recipe} ***\n"
                        "Happy cooking! :)")
                self._wait_start = (
                    self.get_clock().now().seconds_nanoseconds()[0])
                self._last_tick = self._wait_start
                self.get_logger().info("\n--- 20 SECOND WAIT ---")
                self._next = States.WAIT_AFTER_CIRCLE

            case States.WAIT_AFTER_CIRCLE:
                now = self.get_clock().now().seconds_nanoseconds()[0]
                elapsed = now - self._wait_start

                if now - self._last_tick >= 1.0:
                    self.get_logger().info(
                        f"Time remaining: {int(20 - elapsed)}s...")
                    self._last_tick = now

                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.readline().strip().lower() == 'y':
                        self._next = States.IDLE
                        self._cycle_done = True

                if elapsed >= 20:
                    self._next = States.END

            case States.END:
                self.get_logger().warn("Shutdown.")
                raise SystemExit(0)

        if self._state != self._next:
            self.get_logger().info(
                f"TRANSITION: {self._state.name} -> {self._next.name}")
            self._state = self._next


def main(args=None):
    rclpy.init(args=args)
    node = MainStateMachineNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.vision)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        node.vision.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
