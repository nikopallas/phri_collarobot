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

        # Ingredient IDs 6-24
        self.storage = set(range(6, 25))

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
            # self.get_logger().info('Publishing: "%s"' % msg.data) # Removed to avoid interrupting CLI
        except Exception as e:
            self.get_logger().error(f"Failed to get/publish state: {e}")
    
    def cli_loop(self):
        while True:
            try:
                id_input = input("\nEnter ingredient ID (6-24): ").strip()
                if not id_input.isdigit() or not (6 <= int(id_input) <= 24):
                    print("Invalid ID. Please enter a number between 6 and 24.")
                    continue

                target_input = input("Enter target zone (0: storage, 1: proposed, 2: accepted): ").strip()
                if target_input not in ['0', '1', '2']:
                    print("Invalid zone. Use 0, 1, or 2.")
                    continue

                obj_id = int(id_input)
                target = int(target_input)

                self.move_object(obj_id, target)

            except Exception as e:
                print(f"Error in CLI loop: {e}")

    def move_object(self, obj_id: int, target: int):
        """
        target:
            0 = storage
            1 = proposed
            2 = accepted
        """
        if target not in [0, 1, 2]:
            print("Invalid target list. Use 0, 1 or 2.")
            return

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

        print(f"Moved ingredient ID {obj_id} to zone {target}")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = StatePublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
