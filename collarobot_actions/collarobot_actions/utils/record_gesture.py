"""
Interactively record a named gesture as a sequence of joint-state waypoints.

Usage:
  ros2 run collarobot_actions record_gesture <gesture_name> [--description "..."]

Workflow:
  1. Move the arm to the desired pose (via Kinova pendant or teleoperation).
  2. Press [Enter] to snapshot the current joint positions.
  3. Enter the cumulative time for that waypoint (or press [Enter] for +1.0 s default).
  4. Repeat for each waypoint.
  5. Type 'done' / 'd' to write the gesture to gestures.toml.
  6. Type 'undo' / 'u'  to remove the last recorded waypoint.
  7. Type 'quit' / 'q'  to exit without saving.

Times are CUMULATIVE from the start of the gesture (same convention as gestures.toml).

Example session:
  $ ros2 run collarobot_actions record_gesture invite --description "Beckoning gesture"

  Recording gesture "invite" — 0 waypoints so far.
  [Enter]=snapshot  [u]=undo  [d]=done  [q]=quit
  > [Enter]
    Joints: [ 0.0022, -1.2161, -1.3855, -0.0657, -1.3837, -1.5518]
    Cumulative time [+1.0 → 1.0 s]: [Enter]
    Waypoint 1 saved at t=1.0 s

  Recording gesture "invite" — 1 waypoint so far.
  > ...
"""

import argparse
import sys
import threading
import tomllib
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GESTURES_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'gestures.toml'
)


# ---------------------------------------------------------------------------
# TOML helpers
# ---------------------------------------------------------------------------

def _load_gestures() -> dict:
    if GESTURES_PATH.exists():
        with open(GESTURES_PATH, 'rb') as f:
            return tomllib.load(f)
    return {}


def _save_gestures(gestures: dict) -> None:
    lines = []
    for name, data in gestures.items():
        lines.append(f'[{name}]')
        desc = data.get('description', '')
        lines.append(f'description = "{desc}"')
        lines.append('waypoints = [')
        for wp in data['waypoints']:
            wp_str = ', '.join(f'{float(j):.6f}' for j in wp)
            lines.append(f'    [{wp_str}],')
        lines.append(']')
        times_str = ', '.join(f'{float(t):.2f}' for t in data['times'])
        lines.append(f'times = [{times_str}]')
        lines.append('')
    GESTURES_PATH.write_text('\n'.join(lines))


# ---------------------------------------------------------------------------
# ROS node — keeps the latest joint state in _joints
# ---------------------------------------------------------------------------

class JointStateListener(Node):

    def __init__(self):
        super().__init__('record_gesture')
        self._joints = None
        self._lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _on_joints(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))
        missing = [j for j in JOINT_NAMES if j not in joint_map]
        if missing:
            return
        with self._lock:
            self._joints = [joint_map[j] for j in JOINT_NAMES]

    def latest_joints(self):
        with self._lock:
            return list(self._joints) if self._joints is not None else None


# ---------------------------------------------------------------------------
# Interactive recording loop
# ---------------------------------------------------------------------------

def _prompt_time(suggestion: float) -> float:
    """Ask the user for a cumulative time; return suggestion on empty input."""
    while True:
        raw = input(f'    Cumulative time [+1.0 → {suggestion:.1f} s]: ').strip()
        if raw == '':
            return suggestion
        try:
            t = float(raw)
            if t <= 0:
                print('    Time must be > 0.')
                continue
            return t
        except ValueError:
            print('    Enter a number or press Enter for the default.')


def _print_status(gesture_name: str, waypoints: list, times: list):
    n = len(waypoints)
    noun = 'waypoint' if n == 1 else 'waypoints'
    print(f'\nRecording gesture "{gesture_name}" — {n} {noun} so far.')
    for i, (wp, t) in enumerate(zip(waypoints, times), 1):
        joints_str = ', '.join(f'{j:7.4f}' for j in wp)
        print(f'  {i:2d}. [{joints_str}]  t={t:.2f}s')
    print('[Enter]=snapshot  [u]=undo  [d]=done  [q]=quit')


def record_loop(node: JointStateListener, gesture_name: str, description: str):
    """Main interactive loop. Returns (waypoints, times) or (None, None) on quit."""
    waypoints = []
    times = []

    _print_status(gesture_name, waypoints, times)

    while True:
        try:
            cmd = input('> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return None, None

        if cmd in ('q', 'quit'):
            print('Quit — nothing saved.')
            return None, None

        if cmd in ('d', 'done'):
            if not waypoints:
                print('No waypoints recorded yet — nothing to save.')
                continue
            return waypoints, times

        if cmd in ('u', 'undo'):
            if not waypoints:
                print('Nothing to undo.')
            else:
                removed_t = times.pop()
                waypoints.pop()
                print(f'Removed last waypoint (was at t={removed_t:.2f}s).')
                _print_status(gesture_name, waypoints, times)
            continue

        if cmd == '':
            # Snapshot
            joints = node.latest_joints()
            if joints is None:
                print('  Waiting for /joint_states — is the arm running?')
                continue

            joints_str = ', '.join(f'{j:7.4f}' for j in joints)
            print(f'  Joints: [{joints_str}]')

            last_t = times[-1] if times else 0.0
            suggestion = last_t + 1.0
            t = _prompt_time(suggestion)

            if times and t <= times[-1]:
                print(
                    f'  Warning: t={t:.2f}s is not after previous t={times[-1]:.2f}s. '
                    'Saving anyway.'
                )

            waypoints.append(joints)
            times.append(t)
            print(f'  Waypoint {len(waypoints)} saved at t={t:.1f}s')
            _print_status(gesture_name, waypoints, times)
            continue

        print('  Unknown command. Press [Enter] to snapshot, [d]=done, [u]=undo, [q]=quit.')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    parser = argparse.ArgumentParser(
        description='Interactively record a gesture as a sequence of joint waypoints.'
    )
    parser.add_argument('name', help='Gesture name (e.g. invite, present, wave)')
    parser.add_argument(
        '--description', default='',
        metavar='TEXT', help='Human-readable description written to gestures.toml'
    )

    raw = args if args is not None else sys.argv[1:]
    ros_boundary = raw.index('--ros-args') if '--ros-args' in raw else len(raw)
    parsed = parser.parse_args(raw[:ros_boundary])

    rclpy.init(args=args)
    node = JointStateListener()

    # Spin ROS in background so joint-state callbacks keep firing while we block on input().
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gestures = _load_gestures()
    if parsed.name in gestures:
        print(
            f'Gesture "{parsed.name}" already exists in gestures.toml — '
            'it will be overwritten on save.'
        )

    waypoints, times = record_loop(node, parsed.name, parsed.description)

    if waypoints is not None:
        gestures[parsed.name] = {
            'description': parsed.description or parsed.name,
            'waypoints': waypoints,
            'times': times,
        }
        _save_gestures(gestures)
        print(
            f'\nSaved gesture "{parsed.name}" '
            f'({len(waypoints)} waypoints, total {times[-1]:.1f}s) '
            f'to {GESTURES_PATH}'
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
