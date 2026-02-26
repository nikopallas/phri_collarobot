"""
Record a named position (arm joints + carriage + lift) to positions.toml.

Usage:
  ros2 run collarobot_actions record_position <name> [--time SECS]

Examples:
  ros2 run collarobot_actions record_position pre_pick
  ros2 run collarobot_actions record_position pick --time 2.0

Subscribes to:
  /joint_states                     (required)
  /elmo/id2/carriage/position/get   (optional — saved if available)
  /elmo/id2/lift/position/get       (optional — saved if available)
"""

import argparse
import sys
import tomllib
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
POSITIONS_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'positions.toml'
)


def _load_positions() -> dict:
    if POSITIONS_PATH.exists():
        with open(POSITIONS_PATH, 'rb') as f:
            return tomllib.load(f)
    return {}


def _save_positions(positions: dict) -> None:
    lines = []
    # Top-level scalar keys first (gripper_open, gripper_close, ...)
    for k, v in positions.items():
        if not isinstance(v, dict):
            lines.append(f'{k} = {v}')
    if any(not isinstance(v, dict) for v in positions.values()):
        lines.append('')
    # Position tables
    for name, data in positions.items():
        if not isinstance(data, dict):
            continue
        lines.append(f'[{name}]')
        joints_str = ', '.join(f'{float(j):.6f}' for j in data['joints'])
        lines.append(f'joints = [{joints_str}]')
        lines.append(f'time_from_start = {float(data["time_from_start"]):.1f}')
        if 'carriage' in data:
            lines.append(f'carriage = {float(data["carriage"]):.4f}')
        if 'lift' in data:
            lines.append(f'lift = {float(data["lift"]):.4f}')
        lines.append('')
    POSITIONS_PATH.write_text('\n'.join(lines))


class RecordPositionNode(Node):

    def __init__(self, position_name: str, time_from_start: float):
        super().__init__('record_position')
        self._name = position_name
        self._time = time_from_start
        self._done = False
        self._joints = None
        self._carriage = None
        self._lift = None

        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(
            Float32, '/elmo/id2/carriage/position/get', self._on_carriage, 10
        )
        self.create_subscription(
            Float32, '/elmo/id2/lift/position/get', self._on_lift, 10
        )
        self.get_logger().info(
            f'Waiting for /joint_states to record "{position_name}" '
            f'(time_from_start={time_from_start:.1f}s)...'
        )

    def _on_joints(self, msg: JointState):
        if self._joints is not None:
            return  # already captured

        joint_map = dict(zip(msg.name, msg.position))
        missing = [j for j in JOINT_NAMES if j not in joint_map]
        if missing:
            self.get_logger().warn(f'Joint states missing: {missing} — retrying...')
            return

        self._joints = [joint_map[j] for j in JOINT_NAMES]
        self.get_logger().info(
            f'Got joints: [{", ".join(f"{j:.4f}" for j in self._joints)}]'
        )
        # Give carriage/lift topics 0.5 s to deliver, then write.
        self.create_timer(0.5, self._write)

    def _on_carriage(self, msg: Float32):
        self._carriage = msg.data

    def _on_lift(self, msg: Float32):
        self._lift = msg.data

    def _write(self):
        if self._done:
            return
        self._done = True

        entry = {'joints': self._joints, 'time_from_start': self._time}
        if self._carriage is not None:
            entry['carriage'] = self._carriage
            self.get_logger().info(f'  carriage = {self._carriage:.4f}')
        else:
            self.get_logger().info('  carriage: not available (topic not publishing)')
        if self._lift is not None:
            entry['lift'] = self._lift
            self.get_logger().info(f'  lift     = {self._lift:.4f}')
        else:
            self.get_logger().info('  lift: not available (topic not publishing)')

        positions = _load_positions()
        positions[self._name] = entry
        _save_positions(positions)
        self.get_logger().info(f'Saved "{self._name}" to {POSITIONS_PATH}')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Record a named position (joints + carriage + lift) to positions.toml.'
    )
    parser.add_argument('name', help='Position name (e.g. pre_pick, pick, home, ...)')
    parser.add_argument(
        '--time', type=float, default=10.0,
        metavar='SECS', help='time_from_start in seconds (default: 10.0)'
    )

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
