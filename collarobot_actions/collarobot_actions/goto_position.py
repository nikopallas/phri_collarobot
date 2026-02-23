"""
Move the arm to a single named position from positions.toml and exit.

Usage:
  ros2 run collarobot_controller goto_position <name>

Examples:
  ros2 run collarobot_controller goto_position home
  ros2 run collarobot_controller goto_position pre_pick
"""

import argparse
import sys
import time
import tomllib
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
POSITIONS_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'positions.toml'
)


class GotoPositionNode(Node):

    def __init__(self, position_name: str):
        super().__init__('goto_position')

        with open(POSITIONS_PATH, 'rb') as f:
            positions = tomllib.load(f)

        if position_name not in positions:
            available = list(positions.keys())
            self.get_logger().error(
                f'Unknown position "{position_name}". Available: {available}'
            )
            raise SystemExit(1)

        pos = positions[position_name]
        joints = [float(j) for j in pos['joints']]
        duration_sec = float(pos['time_from_start'])

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        # Give the publisher a moment to connect before sending.
        self.create_timer(0.3, lambda: self._send_and_wait(position_name, joints, duration_sec))
        self._sent = False

    def _send_and_wait(self, name: str, joints: list, duration_sec: float):
        if self._sent:
            return
        self._sent = True

        secs = int(duration_sec)
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = Duration(sec=secs, nanosec=int((duration_sec - secs) * 1e9))

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points = [point]

        self._traj_pub.publish(msg)
        self.get_logger().info(
            f'Sent trajectory to "{name}" '
            f'[{", ".join(f"{j:.4f}" for j in joints)}] '
            f'— waiting {duration_sec:.1f}s'
        )
        time.sleep(duration_sec)
        self.get_logger().info(f'Done.')
        raise SystemExit(0)


def main(args=None):
    parser = argparse.ArgumentParser(description='Move arm to a named position.')
    parser.add_argument('name', help='Position name from positions.toml')

    raw = args if args is not None else sys.argv[1:]
    ros_boundary = raw.index('--ros-args') if '--ros-args' in raw else len(raw)
    parsed = parser.parse_args(raw[:ros_boundary])

    rclpy.init(args=args)
    node = GotoPositionNode(parsed.name)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
