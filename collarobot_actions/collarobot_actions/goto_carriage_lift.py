"""
Command carriage and lift to the values recorded for a named position, then exit.

Usage:
  ros2 run collarobot_actions goto_carriage_lift <position_name>

Examples:
  ros2 run collarobot_actions goto_carriage_lift home
  ros2 run collarobot_actions goto_carriage_lift pre_pick

The position must have 'carriage' and/or 'lift' fields in positions.toml
(recorded automatically by record_position when the topics are available).

Publishes to:
  /elmo/id1/carriage/position/set  (std_msgs/Float32)
  /elmo/id1/lift/position/set      (std_msgs/Float32)

Waits until current positions match targets within tolerance, then exits.
Use stop topics for emergency:
  ros2 topic pub --once /elmo/id1/carriage/stop std_msgs/msg/Empty '{}'
  ros2 topic pub --once /elmo/id1/lift/stop     std_msgs/msg/Empty '{}'
"""

import argparse
import sys
import tomllib
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

POSITIONS_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'positions.toml'
)
TOLERANCE = 0.05   # position units — considered "arrived" within this distance
TIMEOUT_SEC = 30.0


class GotoCarriageLiftNode(Node):

    def __init__(self, position_name: str, carriage_target, lift_target):
        super().__init__('goto_carriage_lift')
        self._carriage_target = carriage_target
        self._lift_target = lift_target
        self._carriage_pos = None
        self._lift_pos = None
        self._done = False

        if carriage_target is not None:
            self._carriage_pub = self.create_publisher(
                Float32, '/elmo/id1/carriage/position/set', 10
            )
            self.create_subscription(
                Float32, '/elmo/id1/carriage/position/get', self._on_carriage, 10
            )
        if lift_target is not None:
            self._lift_pub = self.create_publisher(
                Float32, '/elmo/id1/lift/position/set', 10
            )
            self.create_subscription(
                Float32, '/elmo/id1/lift/position/get', self._on_lift, 10
            )

        self._deadline = self.get_clock().now().nanoseconds / 1e9 + TIMEOUT_SEC
        self.create_timer(0.05, self._tick)

        targets = []
        if carriage_target is not None:
            targets.append(f'carriage={carriage_target:.4f}')
        if lift_target is not None:
            targets.append(f'lift={lift_target:.4f}')
        self.get_logger().info(
            f'Moving to "{position_name}": {", ".join(targets)}  '
            f'(timeout {TIMEOUT_SEC:.0f}s, tolerance ±{TOLERANCE})'
        )

    def _on_carriage(self, msg: Float32):
        self._carriage_pos = msg.data

    def _on_lift(self, msg: Float32):
        self._lift_pos = msg.data

    def _tick(self):
        if self._done:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now > self._deadline:
            self.get_logger().error(
                f'Timeout after {TIMEOUT_SEC:.0f}s — carriage/lift did not reach target. '
                'Check hardware or increase TIMEOUT_SEC.'
            )
            self._done = True
            return

        # Publish setpoints on every tick until arrived (idempotent for position controllers).
        if self._carriage_target is not None:
            msg = Float32()
            msg.data = self._carriage_target
            self._carriage_pub.publish(msg)

        if self._lift_target is not None:
            msg = Float32()
            msg.data = self._lift_target
            self._lift_pub.publish(msg)

        # Check arrival
        carriage_ok = (
            self._carriage_target is None
            or (self._carriage_pos is not None
                and abs(self._carriage_pos - self._carriage_target) <= TOLERANCE)
        )
        lift_ok = (
            self._lift_target is None
            or (self._lift_pos is not None
                and abs(self._lift_pos - self._lift_target) <= TOLERANCE)
        )

        if carriage_ok and lift_ok:
            parts = []
            if self._carriage_pos is not None:
                parts.append(f'carriage={self._carriage_pos:.4f}')
            if self._lift_pos is not None:
                parts.append(f'lift={self._lift_pos:.4f}')
            self.get_logger().info(f'Arrived: {", ".join(parts)}')
            self._done = True


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Move carriage and/or lift to values recorded for a named position.'
    )
    parser.add_argument('name', help='Position name from positions.toml')

    raw = args if args is not None else sys.argv[1:]
    ros_boundary = raw.index('--ros-args') if '--ros-args' in raw else len(raw)
    parsed = parser.parse_args(raw[:ros_boundary])

    with open(POSITIONS_PATH, 'rb') as f:
        positions = tomllib.load(f)

    if parsed.name not in positions:
        print(f'Error: position "{parsed.name}" not found in {POSITIONS_PATH}')
        print(f'Available: {[k for k in positions if isinstance(positions[k], dict)]}')
        sys.exit(1)

    pos = positions[parsed.name]
    carriage_target = float(pos['carriage']) if 'carriage' in pos else None
    lift_target = float(pos['lift']) if 'lift' in pos else None

    if carriage_target is None and lift_target is None:
        print(
            f'Error: position "{parsed.name}" has no carriage or lift values recorded.\n'
            'Record them with: ros2 run collarobot_actions record_position '
            f'{parsed.name} --time <secs>'
        )
        sys.exit(1)

    rclpy.init(args=args)
    node = GotoCarriageLiftNode(parsed.name, carriage_target, lift_target)

    while rclpy.ok() and not node._done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
