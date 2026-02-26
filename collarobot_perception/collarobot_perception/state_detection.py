import json
from pathlib import Path
import threading

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

        self.debug = True

        self.storage = set(range(10))   # objects 0–9 start in storage
        self.proposed = set()
        self.accepted = set()

        self.state_lock = threading.Lock()

        self.publisher_ = self.create_publisher(String, 'collarobot/state', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cli_thread = threading.Thread(target=self.cli_loop, daemon=True)
        self.cli_thread.start()

        self.get_logger().info("Mock PublishState node started.")

    def timer_callback(self):
        msg = String()

        if self.debug:
            with self.state_lock:
                state = {
                    "storage": list(self.storage),
                    "proposed": list(self.proposed),
                    "accepted": list(self.accepted),
                    "relative_positions": {}
                }
        else:
            state = capture_and_detect()

        try:
            msg.data = json.dumps(state, indent=4)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to get/publish state: {e}")
    
    def cli_loop(self):
        while True:
            try:
                user_input = input("Move object with format (object_id,target_list): ")

                if not user_input.startswith("(") or not user_input.endswith(")"):
                    print("Invalid format. Example: (9,1)")
                    continue

                obj_id, target = map(int, user_input[1:-1].split(","))

                self.move_object(obj_id, target)

            except Exception as e:
                print(f"Error: {e}")

    def move_object(self, obj_id: int, target: int):
        """
        target:
            0 = storage
            1 = proposed
            2 = accepted
        """
        with self.state_lock:

            # Remove object from all sets
            self.storage.discard(obj_id)
            self.proposed.discard(obj_id)
            self.accepted.discard(obj_id)

            # Add to target
            if target == 0:
                self.storage.add(obj_id)
            elif target == 1:
                self.proposed.add(obj_id)
            elif target == 2:
                self.accepted.add(obj_id)
            else:
                print("Invalid target list. Use 0, 1 or 2.")
                return

        print(f"Moved object {obj_id} to list {target}")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = StatePublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
