import time
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from collarobot_perception.detect_zones import get_state, MarkerDetectionError
from collarobot_perception.take_image import capture_frame

IMAGE_DIR = Path(__file__).parent.parent / "images"
TEST_IMAGE_PATH = IMAGE_DIR / "captured_test.png"


def capture_and_detect(debug=False, max_retries=12) -> dict:
    """Captures a frame and returns the current state with retry logic."""
    for attempt in range(max_retries):
        try:
            frame = capture_frame()
            if frame is None:
                print(f"Attempt {attempt + 1}: Image capture failed.")
                continue

            return get_state(frame, debug=debug)
        except MarkerDetectionError as e:
            print(f"Attempt {attempt + 1}: Marker detection failed: {e}. Retrying...")
            if attempt < max_retries - 1:
                time.sleep(0.1)
            continue
        except Exception as e:
            print(f"Attempt {attempt + 1}: Unexpected error: {e}. Retrying...")
            continue

    raise RuntimeError(f"Failed to detect zones after {max_retries} attempts.")


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(String, 'state', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()

        try:
            state = capture_and_detect()
            msg.data = json.dumps(state, indent=4)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing state')
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
