from collarobot_perception.take_image import capture_frame
from collarobot_perception.detect_zones import get_state as analyze_zones_state
import cv2
from pathlib import Path
import rclpy
from rclpy.node import Node
import json

from std_msgs.msg import String

IMAGE_DIR = Path(__file__).parent.parent / "images"
TEST_IMAGE_PATH = Path("~/collarobot_ws/src/collarobot_perception/images/three_objects.png").expanduser()


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(String, '/collarobot/state', 10)
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


def get_state(debug=False) -> dict:
    """
    Captures a frame from the webcam and returns the current state of markers in zones.
    Returns:
        dict: Dictionary containing the state of markers in zones.
    """
    frame = capture_frame()
    # if frame is None:
    #     print("Error: Could not capture frame from webcam.")
    #     return [], [], []

    frame = cv2.imread(str(TEST_IMAGE_PATH))

    storage, proposed, accepted = analyze_zones_state(frame, debug=debug)

    state = {
        "storage": storage,
        "proposed": proposed,
        "accepted": accepted
    }

    return state


if __name__ == "__main__":
    # Test the real-time detection
    storage, proposed, accepted = get_state(debug=True)
    print("--- Real-time State ---")
    print(f"Storage:  {storage}")
    print(f"Proposed: {proposed}")
    print(f"Accepted: {accepted}")
