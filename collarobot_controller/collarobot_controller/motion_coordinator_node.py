"""
Motion Coordinator Node
=======================
Middle layer between controller_flow and pick_place_node.

- Tracks ingredient positions (exact slot names) in memory.
- Exposes action server /collarobot/move_ingredient (MoveIngredient).
- Resolves (ingredient_id, destination) -> (pick_position, place_position).
- Calls /collarobot/pick_place to execute the physical move.
- Reconciles human moves by subscribing to /collarobot/state.

Slot naming convention: <zone><row>-<col>   e.g. storage1-1, proposal3-2, accepted2-1
  storage  : rows 1-N, cols 1-3  (fixed slot per ingredient from ingredients_mapping.toml)
  proposal : rows 1-4 robot | rows 5-6 human
  accepted : rows 1-4 robot | rows 5-6 human  (2 cols only)

Vision lag handling:
  After a robot move, the coordinator knows both the zone BEFORE and AFTER the move.
  If vision still reports the before-zone → lag, ignore.
  If vision reports the after-zone → caught up, confirm.
  If vision reports something else → genuine human move, process it.

Run:
  ros2 run collarobot_controller motion_coordinator_node
"""

import json
import re
import threading
import tomllib
from pathlib import Path
from typing import Dict, Optional, NamedTuple

import rclpy
from collarobot_msgs.action import MoveIngredient, PickPlace
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

POSITIONS_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'positions.toml'
)
MAPPING_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_controller'
    / 'data' / 'ingredients_mapping.toml'
)
ROBOT_MAX_ROW = 4   # robot owns rows 1-4 in proposal/accepted; human owns rows 5+

# Vision topic uses "proposed"; internal state uses "proposal"
VISION_ZONE_MAP = {
    'storage':  'storage',
    'proposed': 'proposal',
    'accepted': 'accepted',
}


class ExpectedMove(NamedTuple):
    """Tracks what the coordinator expects vision to show after a robot move."""
    before_zone: str   # zone the ingredient was in before the robot moved it
    after_zone: str    # zone the ingredient should be in after the robot moved it
    after_slot: str    # exact slot assigned after the move


def _parse_slot(slot: str):
    """Return (zone, row, col) for a slot name like 'proposal3-2'."""
    m = re.match(r'^([a-z]+)(\d+)-(\d+)$', slot)
    if not m:
        raise ValueError(f'Unparseable slot name: {slot!r}')
    return m.group(1), int(m.group(2)), int(m.group(3))


def _slot_zone(slot: str) -> str:
    zone, _, _ = _parse_slot(slot)
    return zone


def _build_ordered_slots(all_positions: set, zone: str, max_row: Optional[int] = None,
                          min_row: int = 1) -> list:
    """Return slots for *zone* sorted by (row, col), optionally filtered by row range."""
    result = []
    for s in all_positions:
        try:
            z, r, c = _parse_slot(s)
        except ValueError:
            continue
        if z != zone:
            continue
        if r < min_row:
            continue
        if max_row is not None and r > max_row:
            continue
        result.append(s)
    result.sort(key=lambda s: (_parse_slot(s)[1], _parse_slot(s)[2]))
    return result


class MotionCoordinatorNode(Node):

    def __init__(self):
        super().__init__('motion_coordinator_node', namespace='collarobot')

        cb = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # Load positions.toml to discover available named positions
        # ------------------------------------------------------------------
        with open(POSITIONS_PATH, 'rb') as f:
            self._positions = tomllib.load(f)

        # All named position keys (dicts only, excluding top-level scalars)
        all_position_keys = {
            k for k, v in self._positions.items() if isinstance(v, dict)
        }
        self.get_logger().info(f'Loaded {len(all_position_keys)} named positions from positions.toml')

        # ------------------------------------------------------------------
        # Load ingredients mapping
        # ------------------------------------------------------------------
        with open(MAPPING_PATH, 'rb') as f:
            mapping_data = tomllib.load(f)

        # storage_slots: {ingredient_id (int) -> slot_name (str)}
        self._storage_slots: Dict[int, str] = {
            int(k): v for k, v in mapping_data.get('storage_slots', {}).items()
        }

        # ------------------------------------------------------------------
        # Build ordered slot lists for robot and human zones
        # ------------------------------------------------------------------
        # Keys that represent base positions (no pre_ prefix) in proposal/accepted
        base_positions = {
            k for k in all_position_keys
            if not k.startswith('pre_') and k != 'home'
        }

        self._robot_proposal = _build_ordered_slots(base_positions, 'proposal',
                                                     max_row=ROBOT_MAX_ROW)
        self._robot_accepted = _build_ordered_slots(base_positions, 'accepted',
                                                     max_row=ROBOT_MAX_ROW)
        self._human_proposal = _build_ordered_slots(base_positions, 'proposal',
                                                     min_row=ROBOT_MAX_ROW + 1)
        self._human_accepted = _build_ordered_slots(base_positions, 'accepted',
                                                     min_row=ROBOT_MAX_ROW + 1)

        # If no human slots exist yet in positions.toml, pre-generate the expected
        # names (rows 5-6) so tracking works immediately.  The robot can only pick
        # from these once they are physically recorded with record_position.
        if not self._human_proposal:
            self._human_proposal = [
                f'proposal{r}-{c}'
                for r in range(ROBOT_MAX_ROW + 1, ROBOT_MAX_ROW + 3)
                for c in range(1, 4)
            ]
            self.get_logger().warn(
                f'No human proposal slots found in positions.toml — using virtual names '
                f'{self._human_proposal}. Record them to allow robot picks from human zone.'
            )
        if not self._human_accepted:
            self._human_accepted = [
                f'accepted{r}-{c}'
                for r in range(ROBOT_MAX_ROW + 1, ROBOT_MAX_ROW + 3)
                for c in range(1, 3)
            ]
            self.get_logger().warn(
                f'No human accepted slots found in positions.toml — using virtual names '
                f'{self._human_accepted}. Record them to allow robot picks from human zone.'
            )

        self.get_logger().info(
            f'Robot proposal slots ({len(self._robot_proposal)}): {self._robot_proposal}'
        )
        self.get_logger().info(
            f'Robot accepted slots ({len(self._robot_accepted)}): {self._robot_accepted}'
        )

        # ------------------------------------------------------------------
        # Initialize ingredient position tracking
        # ------------------------------------------------------------------
        # ingredient_positions: ingredient_id -> current slot name (or None = unknown)
        self._ingredient_positions: Dict[int, Optional[str]] = {}
        # slot_occupancy: slot_name -> ingredient_id (or None = empty)
        self._slot_occupancy: Dict[str, Optional[int]] = {}

        for ing_id, slot in self._storage_slots.items():
            if slot not in all_position_keys:
                self.get_logger().warn(
                    f'Ingredient {ing_id}: storage slot "{slot}" not in positions.toml — '
                    f'skipping (record it to enable)'
                )
                self._ingredient_positions[ing_id] = None
            else:
                self._ingredient_positions[ing_id] = slot
                self._slot_occupancy[slot] = ing_id

        self.get_logger().info(
            f'Initialized {sum(1 for v in self._ingredient_positions.values() if v)} '
            f'ingredients with known positions'
        )

        # Ingredient being moved by robot right now (suppresses vision reconciliation)
        self._robot_moving_ingredient: Optional[int] = None

        # Expected moves: after a robot move completes, we know what zones vision
        # should transition through.  This lets us distinguish "vision lag" from
        # "genuine human move" without any timers.
        # Key: ingredient_id, Value: ExpectedMove(before_zone, after_zone, after_slot)
        self._expected_moves: Dict[int, ExpectedMove] = {}

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.create_subscription(
            String, '/collarobot/state', self._vision_callback, 10,
            callback_group=cb,
        )

        self._pick_place_client = ActionClient(
            self, PickPlace, '/collarobot/pick_place', callback_group=cb,
        )

        self._action_server = ActionServer(
            self,
            MoveIngredient,
            'move_ingredient',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb,
        )

        self.get_logger().info('Ready.')

    # ------------------------------------------------------------------
    # Slot helpers
    # ------------------------------------------------------------------

    def _next_robot_slot(self, zone: str) -> Optional[str]:
        slots = self._robot_proposal if zone == 'proposal' else self._robot_accepted
        for slot in slots:
            if self._slot_occupancy.get(slot) is None:
                return slot
        return None

    def _next_human_slot(self, zone: str) -> Optional[str]:
        slots = self._human_proposal if zone == 'proposal' else self._human_accepted
        for slot in slots:
            if self._slot_occupancy.get(slot) is None:
                return slot
        return None

    def _free_slot(self, slot: Optional[str]):
        if slot and slot in self._slot_occupancy:
            self._slot_occupancy[slot] = None

    def _occupy_slot(self, slot: str, ing_id: int):
        self._slot_occupancy[slot] = ing_id

    # ------------------------------------------------------------------
    # Vision reconciliation
    # ------------------------------------------------------------------

    def _vision_callback(self, msg: String):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse vision state: {e}')
            return

        # Build vision zone membership: ingredient_id -> zone ('storage'/'proposal'/'accepted')
        vision_zone: Dict[int, str] = {}
        for vision_key, internal_zone in VISION_ZONE_MAP.items():
            for ing_id in state.get(vision_key, []):
                vision_zone[int(ing_id)] = internal_zone

        for ing_id, current_slot in list(self._ingredient_positions.items()):
            if ing_id == self._robot_moving_ingredient:
                continue  # robot is handling this one

            current_zone = _slot_zone(current_slot) if current_slot else None
            new_zone = vision_zone.get(ing_id)

            if new_zone is None or new_zone == current_zone:
                continue  # no change detected

            # Check if this ingredient has a pending expected move
            expected = self._expected_moves.get(ing_id)
            if expected is not None:
                if new_zone == expected.before_zone:
                    # Vision still shows the old zone → lag, ignore
                    self.get_logger().debug(
                        f'Vision lag: ingredient {ing_id} still shows '
                        f'{new_zone!r} (expected transition to {expected.after_zone!r})'
                    )
                    continue
                if new_zone == expected.after_zone:
                    # Vision caught up — confirm the robot's move
                    self.get_logger().info(
                        f'Vision confirmed: ingredient {ing_id} now in '
                        f'{new_zone!r} (slot {expected.after_slot!r})'
                    )
                    del self._expected_moves[ing_id]
                    continue
                # Vision shows a THIRD zone — genuine human move after robot move
                self.get_logger().info(
                    f'Unexpected zone for ingredient {ing_id}: expected '
                    f'{expected.after_zone!r}, vision shows {new_zone!r} — '
                    f'treating as human move'
                )
                del self._expected_moves[ing_id]
                # Fall through to human-move handling below

            # Human moved this ingredient
            self.get_logger().info(
                f'Human move detected: ingredient {ing_id} '
                f'{current_slot!r} ({current_zone}) -> zone {new_zone!r}'
            )
            self._free_slot(current_slot)

            if new_zone == 'storage':
                new_slot = self._storage_slots.get(ing_id)
            elif new_zone in ('proposal', 'accepted'):
                new_slot = self._next_human_slot(new_zone)
                if new_slot is None:
                    self.get_logger().warn(
                        f'Human moved ingredient {ing_id} to {new_zone} but no free human slot'
                    )
            else:
                new_slot = None

            self._ingredient_positions[ing_id] = new_slot
            if new_slot:
                self._occupy_slot(new_slot, ing_id)

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        ingredient_id = goal_handle.request.ingredient_id
        destination = goal_handle.request.destination
        result = MoveIngredient.Result()

        self.get_logger().info(
            f'Move request: ingredient {ingredient_id} -> {destination}'
        )

        # Resolve current slot
        current_slot = self._ingredient_positions.get(ingredient_id)
        if current_slot is None:
            msg = f'Unknown position for ingredient {ingredient_id}'
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result

        # Resolve destination slot
        if destination == 'storage':
            place_slot = self._storage_slots.get(ingredient_id)
            if place_slot is None:
                msg = f'No storage slot mapping for ingredient {ingredient_id}'
                self.get_logger().error(msg)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result
        elif destination in ('proposal', 'accepted'):
            place_slot = self._next_robot_slot(destination)
            if place_slot is None:
                msg = f'No free robot slot in {destination} zone'
                self.get_logger().error(msg)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result
        else:
            msg = f'Unknown destination: {destination!r}'
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result

        # Validate both positions exist in positions.toml
        for pos in (current_slot, place_slot):
            if pos not in self._positions:
                msg = f'Position "{pos}" not found in positions.toml'
                self.get_logger().error(msg)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result

        before_zone = _slot_zone(current_slot)
        after_zone = _slot_zone(place_slot)

        self.get_logger().info(
            f'  pick="{current_slot}" ({before_zone})  '
            f'place="{place_slot}" ({after_zone})'
        )

        # Suppress vision reconciliation for this ingredient during move
        self._robot_moving_ingredient = ingredient_id

        # Publish feedback
        fb = MoveIngredient.Feedback()
        fb.state = 'CALLING_PICK_PLACE'
        goal_handle.publish_feedback(fb)

        # Call pick_place action
        success, error_msg = self._call_pick_place(current_slot, place_slot)

        if success:
            # Update internal tracking
            self._free_slot(current_slot)
            self._ingredient_positions[ingredient_id] = place_slot
            self._occupy_slot(place_slot, ingredient_id)
            self.get_logger().info(
                f'  ingredient {ingredient_id} moved to {place_slot}'
            )
            goal_handle.succeed()
            result.success = True
            result.message = 'OK'
            result.pick_position = current_slot
            result.place_position = place_slot

            # Register expected move so vision callback can distinguish
            # "lag" (still shows before_zone) from "human move" (third zone)
            self._expected_moves[ingredient_id] = ExpectedMove(
                before_zone=before_zone,
                after_zone=after_zone,
                after_slot=place_slot,
            )
            self.get_logger().info(
                f'  expecting vision: {before_zone!r} -> {after_zone!r}'
            )
        else:
            self.get_logger().error(f'pick_place failed: {error_msg}')
            goal_handle.abort()
            result.success = False
            result.message = error_msg

        self._robot_moving_ingredient = None
        return result

    # ------------------------------------------------------------------
    # Pick-place action client helper
    # ------------------------------------------------------------------

    def _call_pick_place(self, pick_pos: str, place_pos: str):
        """Send a PickPlace goal and block until it completes.

        Returns (success: bool, message: str).
        Uses threading.Event + add_done_callback — never spin_until_future_complete.
        """
        if not self._pick_place_client.wait_for_server(timeout_sec=10.0):
            return False, 'pick_place action server not available'

        goal = PickPlace.Goal()
        goal.pick_position = pick_pos
        goal.place_position = place_pos
        goal.ignore_carriage = False

        done = threading.Event()
        outcome = {}

        def _on_goal(future):
            handle = future.result()
            if not handle.accepted:
                outcome['success'] = False
                outcome['message'] = 'pick_place goal rejected'
                done.set()
                return

            def _on_result(res_future):
                res = res_future.result().result
                outcome['success'] = res.success
                outcome['message'] = res.message
                done.set()

            handle.get_result_async().add_done_callback(_on_result)

        self._pick_place_client.send_goal_async(goal).add_done_callback(_on_goal)

        # Wait (pick_place can take up to ~60 s for a full sequence)
        if not done.wait(timeout=120.0):
            return False, 'pick_place timed out after 120 s'

        return outcome.get('success', False), outcome.get('message', '')


def main(args=None):
    rclpy.init(args=args)
    node = MotionCoordinatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
