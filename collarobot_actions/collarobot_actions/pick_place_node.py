"""
Pick-and-place action server for Kinova Gen3 + Robotiq gripper.

Sequence (any exception → action aborted):
  PRE_PICK → PICKING → POST_PICK → HOME → PRE_PLACE → PLACING → POST_PLACE

  POST_PICK  reuses the pre_pick  position (retreat to approach height)
  POST_PLACE reuses the pre_place position (retreat to approach height)
  HOME is a safe neutral transit pose between the pick and place zones.

Before starting, the node verifies that carriage and lift are within tolerance of
the values recorded in the 'home' position (if carriage/lift were recorded there).
The operator must set carriage and lift manually; use goto_carriage_lift to command them.
Gripper values are read from positions.toml (gripper_open / gripper_close).

Action client:
  ros2 action send_goal --feedback /pick_place_node/pick_place \\
      collarobot_msgs/action/PickPlace '{}'
"""

import threading
import time
import tomllib
from enum import Enum, auto
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from collarobot_msgs.action import PickPlace
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
CARRIAGE_LIFT_TOL = 0.05  # tolerance for carriage/lift position check (same units as position/get)
POSITIONS_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'positions.toml'
)


class Step(Enum):
    PRE_PICK = auto()
    PICKING = auto()
    POST_PICK = auto()
    HOME = auto()
    PRE_PLACE = auto()
    PLACING = auto()
    POST_PLACE = auto()


SEQUENCE = [
    Step.PRE_PICK,
    Step.PICKING,
    Step.POST_PICK,
    Step.HOME,
    Step.PRE_PLACE,
    Step.PLACING,
    Step.POST_PLACE,
]


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_place_node')

        with open(POSITIONS_PATH, 'rb') as f:
            self._positions = tomllib.load(f)
        self._gripper_open = float(self._positions.get('gripper_open', 0.0))
        self._gripper_close = float(self._positions.get('gripper_close', 0.8))
        named = [k for k in self._positions if isinstance(self._positions[k], dict)]
        self.get_logger().info(
            f'Loaded positions: {named}  '
            f'gripper open={self._gripper_open} close={self._gripper_close}'
        )

        # ReentrantCallbackGroup lets the action execute_callback and the
        # gripper action client callbacks run concurrently in MultiThreadedExecutor.
        cb = ReentrantCallbackGroup()

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        # Subscribe to carriage/lift feedback for pre-run position check.
        self._carriage_pos = None
        self._lift_pos = None
        self.create_subscription(
            Float32, '/elmo/id1/carriage/position/get', self._on_carriage, 10
        )
        self.create_subscription(
            Float32, '/elmo/id1/lift/position/get', self._on_lift, 10
        )

        self._gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd',
            callback_group=cb,
        )

        self._busy = False
        self._busy_lock = threading.Lock()

        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb,
        )

        self.get_logger().info('Ready.')

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Rejecting goal: already running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Pick-and-place started')
        result = PickPlace.Result()
        step = None
        try:
            self._check_setup()
            for step in SEQUENCE:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Cancelled')
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled'
                    return result

                fb = PickPlace.Feedback()
                fb.state = step.name
                goal_handle.publish_feedback(fb)
                self.get_logger().info(f'Step -> {step.name}')
                self._execute_step(step)

            goal_handle.succeed()
            result.success = True
            result.message = 'Pick-and-place complete'

        except Exception as exc:
            where = step.name if step is not None else 'setup check'
            self.get_logger().error(f'Error in {where}: {exc}')
            goal_handle.abort()
            result.success = False
            result.message = str(exc)

        finally:
            with self._busy_lock:
                self._busy = False

        return result

    # ------------------------------------------------------------------
    # Carriage / lift monitoring
    # ------------------------------------------------------------------

    def _on_carriage(self, msg: Float32):
        self._carriage_pos = msg.data

    def _on_lift(self, msg: Float32):
        self._lift_pos = msg.data

    def _check_setup(self):
        """Verify carriage and lift are at the positions recorded for 'home'.

        Raises RuntimeError with a descriptive message if any axis is out of
        tolerance so the operator knows what to adjust.  If carriage/lift were
        not recorded in the home entry, the check is silently skipped.
        """
        home = self._positions.get('home', {})
        issues = []

        for axis, stored_key, current in [
            ('carriage', 'carriage', self._carriage_pos),
            ('lift',     'lift',     self._lift_pos),
        ]:
            if stored_key not in home:
                continue  # not recorded for this setup — skip
            expected = float(home[stored_key])
            if current is None:
                self.get_logger().warn(
                    f'{axis} position unknown (no data on topic yet) — skipping check'
                )
                continue
            if abs(current - expected) > CARRIAGE_LIFT_TOL:
                issues.append(
                    f'{axis}: expected {expected:.4f}, got {current:.4f} '
                    f'(tolerance ±{CARRIAGE_LIFT_TOL})'
                )

        if issues:
            raise RuntimeError(
                'Setup check failed — adjust manually before triggering:\n  '
                + '\n  '.join(issues)
            )

    # ------------------------------------------------------------------
    # Step execution
    # ------------------------------------------------------------------

    def _execute_step(self, step: Step):
        if step == Step.PRE_PICK:
            self._move_to('pre_pick')
        elif step == Step.PICKING:
            self._move_to('pick')
            self._gripper(self._gripper_close)
        elif step == Step.POST_PICK:
            self._move_to('pre_pick')   # retreat to approach height (same position)
        elif step == Step.HOME:
            self._move_to('home')
        elif step == Step.PRE_PLACE:
            self._move_to('pre_place')
        elif step == Step.PLACING:
            self._move_to('place')
            self._gripper(self._gripper_open)
        elif step == Step.POST_PLACE:
            self._move_to('pre_place')  # retreat to approach height (same position)

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def _move_to(self, name: str):
        pos = self._positions[name]
        joints = pos['joints']
        duration_sec = float(pos['time_from_start'])

        # Arm trajectory
        point = JointTrajectoryPoint()
        point.positions = [float(j) for j in joints]
        secs = int(duration_sec)
        point.time_from_start = Duration(sec=secs, nanosec=int((duration_sec - secs) * 1e9))

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points = [point]

        deadline = time.time() + 5.0
        while self._traj_pub.get_subscription_count() == 0:
            if time.time() > deadline:
                raise RuntimeError('Trajectory controller not connected (no subscribers)')
            time.sleep(0.05)

        self._traj_pub.publish(msg)
        self.get_logger().info(f'Moving to "{name}" — waiting {duration_sec:.1f}s')
        time.sleep(duration_sec)

    def _gripper(self, position: float):
        label = 'close' if position > 0.1 else 'open'
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Gripper action server not available')

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 50.0

        done = threading.Event()

        def _on_goal(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error('Gripper goal rejected')
                done.set()
                return
            handle.get_result_async().add_done_callback(lambda _f: done.set())

        self._gripper_client.send_goal_async(goal).add_done_callback(_on_goal)

        if not done.wait(timeout=10.0):
            raise RuntimeError(f'Gripper {label} timed out')
        self.get_logger().info(f'Gripper {label} done')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
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
