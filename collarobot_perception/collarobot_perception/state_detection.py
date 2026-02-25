

from collarobot_perception.detect_zones import get_state as analyze_zones_state

import cv2
from pathlib import Path
import rclpy
from rclpy.node import Node
import json

from std_msgs.msg import String

IMAGE_DIR = Path(__file__).parent.parent / "images"
TEST_IMAGE_PATH = IMAGE_DIR / "captured_test.png"


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(String, 'state', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()

        msg.data = json.dumps(get_state(), indent=4)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = StatePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


def get_state(debug=False) -> dict:
    """Captures a frame and returns the current state of markers in zones."""
    frame = cv2.imread(str(TEST_IMAGE_PATH))
    storage, proposed, accepted, relative_positions = analyze_zones_state(frame, debug=debug)

    return {
        "storage": storage,
        "proposed": proposed,
        "accepted": accepted,
        "relative_positions": relative_positions
    }


if __name__ == "__main__":
    state = get_state(debug=True)
    print("--- Real-time State ---")
    for k, v in state.items():
        print(f"{k.capitalize()}: {v}")
