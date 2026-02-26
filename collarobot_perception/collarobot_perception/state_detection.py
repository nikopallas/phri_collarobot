import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from collarobot_perception.detect_zones import get_state
from collarobot_perception.take_image import capture_frame

IMAGE_DIR = Path(__file__).parent.parent / "images"
TEST_IMAGE_PATH = Path("~/collarobot_ws/src/collarobot_perception/images/capture").expanduser()


def capture_and_detect(debug=False) -> dict:
    """Captures a frame and returns the current state."""
    frame = capture_frame()
    if frame is None:
        raise RuntimeError("Image capture failed.")
    return get_state(frame, debug=debug)


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(String, 'collarobot/state', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()

        try:
            state = capture_and_detect()
            msg.data = json.dumps(state, indent=4)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to get/publish state: {e}")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = StatePublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
