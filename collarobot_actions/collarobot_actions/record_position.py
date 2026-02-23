"""
Record a named joint position from the live robot and save it to positions.toml.

Usage:
  ros2 run collarobot_actions record_position <name> [--time SECS]

Examples:
  ros2 run collarobot_actions record_position pre_pick
  ros2 run collarobot_actions record_position pick --time 2.0
"""

import argparse
import sys
import tomllib
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
POSITIONS_PATH = Path(__file__).parent / 'positions.toml'


def _load_positions() -> dict:
    if POSITIONS_PATH.exists():
        with open(POSITIONS_PATH, 'rb') as f:
            return tomllib.load(f)
    return {}


def _save_positions(positions: dict) -> None:
    lines = []
    for name, data in positions.items():
        lines.append(f'[{name}]')
        joints_str = ', '.join(f'{float(j):.6f}' for j in data['joints'])
        lines.append(f'joints = [{joints_str}]')
        lines.append(f'time_from_start = {float(data["time_from_start"]):.1f}')
        lines.append('')
    POSITIONS_PATH.write_text('\n'.join(lines))


class RecordPositionNode(Node):

    def __init__(self, position_name: str, time_from_start: float):
        super().__init__('record_position')
        self._name = position_name
        self._time = time_from_start
        self._done = False

        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.get_logger().info(
            f'Waiting for /joint_states to record "{position_name}" '
            f'(time_from_start={time_from_start:.1f}s)...'
        )

    def _cb(self, msg: JointState):
        if self._done:
            return

        joint_map = dict(zip(msg.name, msg.position))
        missing = [j for j in JOINT_NAMES if j not in joint_map]
        if missing:
            self.get_logger().warn(f'Joint states missing: {missing} — retrying...')
            return

        joints = [joint_map[j] for j in JOINT_NAMES]

        positions = _load_positions()
        positions[self._name] = {'joints': joints, 'time_from_start': self._time}
        _save_positions(positions)

        self.get_logger().info(
            f'Saved "{self._name}": [{", ".join(f"{j:.4f}" for j in joints)}]'
        )
        self._done = True


def main(args=None):
    # Parse our own args before handing the rest to rclpy.
    parser = argparse.ArgumentParser(
        description='Record a named joint position to positions.toml.'
    )
    parser.add_argument('name', help='Position name (e.g. pre_pick, pick, place, ...)')
    parser.add_argument(
        '--time', type=float, default=3.0,
        metavar='SECS', help='time_from_start in seconds (default: 3.0)'
    )

    # Strip ROS-specific args before argparse sees them.
    raw = args if args is not None else sys.argv[1:]
    ros_boundary = raw.index('--ros-args') if '--ros-args' in raw else len(raw)
    parsed = parser.parse_args(raw[:ros_boundary])

    rclpy.init(args=args)
    node = RecordPositionNode(parsed.name, parsed.time)

    while rclpy.ok() and not node._done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
