"""
Gesture action server for Kinova Gen3 arm expressions.

Sends all waypoints of a named gesture as a single multi-point JointTrajectory
so the controller interpolates smoothly between them.

Gestures are defined in gestures.toml (same directory as positions.toml).

Action server: /gesture_node/gesture  (collarobot_msgs/action/Gesture)

Usage:
  ros2 run collarobot_actions gesture_node

  ros2 action send_goal --feedback /gesture_node/gesture \\
      collarobot_msgs/action/Gesture \\
      '{gesture_name: "wiggle", return_position: "home"}'
"""

import threading
import time
import tomllib
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from collarobot_msgs.action import Gesture
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

GESTURES_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'gestures.toml'
)
POSITIONS_PATH = (
    Path.home() / 'collarobot_ws' / 'src' / 'collarobot_actions'
    / 'collarobot_actions' / 'positions.toml'
)


class GestureNode(Node):

    def __init__(self):
        super().__init__('gesture_node', namespace='collarobot')

        with open(GESTURES_PATH, 'rb') as f:
            self._gestures = tomllib.load(f)
        with open(POSITIONS_PATH, 'rb') as f:
            self._positions = tomllib.load(f)

        self.get_logger().info(
            f'Loaded gestures: {list(self._gestures.keys())}'
        )

        cb = ReentrantCallbackGroup()

        self._traj_pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10
        )

        self._busy = False
        self._busy_lock = threading.Lock()

        self._action_server = ActionServer(
            self,
            Gesture,
            'gesture',
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
            if goal_request.gesture_name not in self._gestures:
                self.get_logger().warn(
                    f'Rejecting goal: unknown gesture "{goal_request.gesture_name}". '
                    f'Available: {list(self._gestures.keys())}'
                )
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        gesture_name = goal_handle.request.gesture_name
        return_position = goal_handle.request.return_position
        self.get_logger().info(
            f'Gesture "{gesture_name}" started'
            + (f' (return to "{return_position}")' if return_position else '')
        )

        result = Gesture.Result()
        try:
            gesture = self._gestures[gesture_name]
            waypoints = gesture['waypoints']
            times = gesture['times']

            # --- Feedback: EXECUTING ---
            fb = Gesture.Feedback()
            fb.state = 'EXECUTING'
            fb.waypoint_index = 0
            fb.waypoint_count = len(waypoints)
            goal_handle.publish_feedback(fb)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                return result

            self._execute_gesture(waypoints, times)

            # --- Feedback: RETURNING ---
            if return_position:
                if return_position not in self._positions:
                    raise RuntimeError(
                        f'return_position "{return_position}" not found in positions.toml'
                    )
                time.sleep(1.0)  # settle after gesture before moving back
                fb.state = 'RETURNING'
                goal_handle.publish_feedback(fb)
                self._move_to(return_position)

            goal_handle.succeed()
            result.success = True
            result.message = f'Gesture "{gesture_name}" complete'

        except Exception as exc:
            self.get_logger().error(f'Error in gesture "{gesture_name}": {exc}')
            goal_handle.abort()
            result.success = False
            result.message = str(exc)

        finally:
            with self._busy_lock:
                self._busy = False

        return result

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def _execute_gesture(self, waypoints, times):
        """Send all waypoints as a single JointTrajectory and sleep for the total duration."""
        points = []
        for joints, t in zip(waypoints, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(j) for j in joints]
            secs = int(t)
            pt.time_from_start = Duration(sec=secs, nanosec=int((t - secs) * 1e9))
            points.append(pt)

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points = points

        deadline = time.time() + 5.0
        while self._traj_pub.get_subscription_count() == 0:
            if time.time() > deadline:
                raise RuntimeError('Trajectory controller not connected (no subscribers)')
            time.sleep(0.05)

        self._traj_pub.publish(msg)
        total_duration = float(times[-1])
        self.get_logger().info(
            f'Executing gesture — {len(points)} waypoints, '
            f'total duration {total_duration:.1f}s'
        )
        time.sleep(total_duration)

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

        deadline = time.time() + 5.0
        while self._traj_pub.get_subscription_count() == 0:
            if time.time() > deadline:
                raise RuntimeError('Trajectory controller not connected (no subscribers)')
            time.sleep(0.05)

        self._traj_pub.publish(msg)
        self.get_logger().info(f'Moving to "{name}" — waiting {duration_sec:.1f}s')
        time.sleep(duration_sec)


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
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
