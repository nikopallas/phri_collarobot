"""
Send a single GripperCommand goal and exit.  Use this to calibrate grip positions.

Usage:
  ros2 run collarobot_actions test_gripper <position> [--effort NEWTONS]

  position  0.0 = fully open, 1.0 = fully closed (normalised scale)

Examples:
  ros2 run collarobot_actions test_gripper 0.0          # open
  ros2 run collarobot_actions test_gripper 0.45         # ~5 cm cube
  ros2 run collarobot_actions test_gripper 1.0          # fully closed
  ros2 run collarobot_actions test_gripper 0.45 --effort 30
"""

import argparse
import sys

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node

GRIPPER_ACTION = '/robotiq_gripper_controller/gripper_cmd'
DEFAULT_EFFORT = 50.0


class TestGripperNode(Node):

    def __init__(self, position: float, effort: float):
        super().__init__('test_gripper')
        self._client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self._position = position
        self._effort = effort
        self._done = False
        self.create_timer(0.1, self._try_send)
        self._sent = False

    def _try_send(self):
        if self._sent:
            return
        if not self._client.server_is_ready():
            self.get_logger().info('Waiting for gripper action server...', once=True)
            return
        self._sent = True

        goal = GripperCommand.Goal()
        goal.command.position = self._position
        goal.command.max_effort = self._effort

        self.get_logger().info(
            f'Sending gripper position={self._position:.3f}  effort={self._effort:.1f}N'
        )
        self._client.send_goal_async(goal).add_done_callback(self._on_goal)

    def _on_goal(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by gripper action server')
            self._done = True
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f'Done — reached={result.reached_goal}  '
            f'position={result.position:.4f}  '
            f'effort={result.effort:.2f}N'
        )
        self._done = True


def main(args=None):
    parser = argparse.ArgumentParser(description='Test gripper at a specific position.')
    parser.add_argument(
        'position', type=float,
        help='Gripper position (0.0 = open, 1.0 = fully closed)'
    )
    parser.add_argument(
        '--effort', type=float, default=DEFAULT_EFFORT,
        metavar='N', help=f'Max effort in Newtons (default: {DEFAULT_EFFORT})'
    )

    raw = args if args is not None else sys.argv[1:]
    ros_boundary = raw.index('--ros-args') if '--ros-args' in raw else len(raw)
    parsed = parser.parse_args(raw[:ros_boundary])

    if not 0.0 <= parsed.position <= 1.0:
        print(f'Error: position must be between 0.0 and 1.0 (got {parsed.position})')
        sys.exit(1)

    rclpy.init(args=args)
    node = TestGripperNode(parsed.position, parsed.effort)

    while rclpy.ok() and not node._done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
