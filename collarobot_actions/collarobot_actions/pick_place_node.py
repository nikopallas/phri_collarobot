"""
Pick-and-place state machine node for Kinova Gen3 + Robotiq gripper.

State transitions (strict linear, any exception -> ERROR -> IDLE):
  IDLE -> PRE_PICK -> PICKING -> POST_PICK -> PRE_PLACE -> PLACING -> POST_PLACE -> IDLE

Trigger:  ros2 service call /pick_place_node/trigger std_srvs/srv/Trigger
State:    ros2 topic echo /pick_place_node/state
"""

import threading
import time
import tomllib
from enum import Enum, auto
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.8
POSITIONS_PATH = Path(__file__).resolve().parent / 'positions.toml'


class State(Enum):
    IDLE = auto()
    PRE_PICK = auto()
    PICKING = auto()
    POST_PICK = auto()
    PRE_PLACE = auto()
    PLACING = auto()
    POST_PLACE = auto()
    ERROR = auto()


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_place_node')

        with open(POSITIONS_PATH, 'rb') as f:
            self._positions = tomllib.load(f)
        self.get_logger().info(f'Loaded positions: {list(self._positions.keys())}')

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self._state_pub = self.create_publisher(String, '~/state', 10)

        self._gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd'
        )

        self.create_service(Trigger, '~/trigger', self._trigger_callback)
        self.create_timer(0.5, self._publish_state)

        self._state = State.IDLE
        self._lock = threading.Lock()

        self.get_logger().info('Ready. Call ~/trigger to start pick-and-place.')

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------

    def _trigger_callback(self, request, response):
        with self._lock:
            if self._state != State.IDLE:
                response.success = False
                response.message = f'Not idle (current state: {self._state.name})'
                return response
            self._state = State.PRE_PICK

        threading.Thread(target=self._run_state_machine, daemon=True).start()
        response.success = True
        response.message = 'Pick-and-place started'
        return response

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _run_state_machine(self):
        try:
            self._execute(State.PRE_PICK)
            self._execute(State.PICKING)
            self._execute(State.POST_PICK)
            self._execute(State.PRE_PLACE)
            self._execute(State.PLACING)
            self._execute(State.POST_PLACE)
            self._set_state(State.IDLE)
        except Exception as exc:
            self.get_logger().error(f'Error in {self._state.name}: {exc}')
            self._set_state(State.ERROR)
            time.sleep(1.0)
            self._set_state(State.IDLE)

    def _execute(self, state: State):
        self._set_state(state)

        if state == State.PRE_PICK:
            self._move_to('pre_pick')
        elif state == State.PICKING:
            self._move_to('pick')
            self._gripper(GRIPPER_CLOSE)
        elif state == State.POST_PICK:
            self._move_to('post_pick')
        elif state == State.PRE_PLACE:
            self._move_to('pre_place')
        elif state == State.PLACING:
            self._move_to('place')
            self._gripper(GRIPPER_OPEN)
        elif state == State.POST_PLACE:
            self._move_to('post_place')

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def _move_to(self, name: str):
        pos = self._positions[name]
        joints = pos['joints']
        duration_sec = float(pos['time_from_start'])

        point = JointTrajectoryPoint()
        point.positions = [float(j) for j in joints]
        secs = int(duration_sec)
        point.time_from_start = Duration(sec=secs, nanosec=int((duration_sec - secs) * 1e9))

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points = [point]

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

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    def _set_state(self, state: State):
        self._state = state
        self.get_logger().info(f'State -> {state.name}')
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)


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
