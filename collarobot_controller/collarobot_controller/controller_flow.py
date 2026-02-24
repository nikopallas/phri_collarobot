import rclpy
from rclpy.node import Node
from enum import Enum
import time
import sys
import select

# Import your model
# Ensure models.py is in the same directory or your PYTHONPATH
try:
    from .models import pick_action
except ImportError:
    # Fallback if not running as a package
    import models


class States(Enum):
    IDLE = 0
    WAIT_UNTIL_HUMAN_PUT_INGREDIENT = 1
    MAIN_BRAIN = 2
    PROPOSE_NEW_INGREDIENT = 3
    PUT_INGREDIENT_BACK = 4
    ACCEPT_INGREDIENT = 5
    HAND_INVITATION = 6
    HAND_CIRCLE = 7
    WAIT_AFTER_CIRCLE = 8
    END = 9


class MainStateMachineNode(Node):
    def __init__(self):
        super().__init__('main_state_machine')

        self.current_state = States.IDLE
        self.next_state = States.IDLE
        self.cycle_done = True
        self.wait_start_time = 0.0
        self.last_tick_time = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Node started with models.py integration.")

    def subscribe_from_vision(self) -> bool:
        # Simulator prompt
        val = input(f"[VISION] Ingredient detected? (y/n): ").lower()
        return val == 'y'

    def execute_motion(self, motion_name):
        self.get_logger().info(f">>> STARTING MOTION: {motion_name}")
        time.sleep(1.0)

    def timer_callback(self):
        match self.current_state:
            case States.IDLE:
                if self.cycle_done:
                    self.cycle_done = False
                    self.next_state = States.HAND_INVITATION

            case States.HAND_INVITATION:
                self.execute_motion("Hand Invitation")
                if self.subscribe_from_vision():
                    self.next_state = States.MAIN_BRAIN
                else:
                    self.next_state = States.WAIT_UNTIL_HUMAN_PUT_INGREDIENT

            case States.WAIT_UNTIL_HUMAN_PUT_INGREDIENT:
                if self.subscribe_from_vision():
                    self.next_state = States.MAIN_BRAIN

            case States.MAIN_BRAIN:
                # --- INTEGRATION WITH models.py ---
                # We call pick_action() and get the string
                decision = models.pick_action()
                self.get_logger().info(f"Model Decision: {decision}")

                if decision == "proposed":
                    self.next_state = States.PROPOSE_NEW_INGREDIENT
                elif decision == "rejected":
                    self.next_state = States.PUT_INGREDIENT_BACK
                elif decision == "accepted":
                    self.next_state = States.ACCEPT_INGREDIENT
                elif decision == "skip":
                    self.next_state = States.HAND_CIRCLE
                else:
                    self.get_logger().warn(f"Unknown decision: {decision}. Staying in Brain.")

            case States.PROPOSE_NEW_INGREDIENT:
                self.execute_motion("Propose New")
                self.next_state = States.MAIN_BRAIN

            case States.PUT_INGREDIENT_BACK:
                self.execute_motion("Put Back")
                self.next_state = States.MAIN_BRAIN

            case States.ACCEPT_INGREDIENT:
                self.execute_motion("Accept")
                self.next_state = States.MAIN_BRAIN

            case States.HAND_CIRCLE:
                self.execute_motion("Hand Circle")
                self.wait_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.last_tick_time = self.wait_start_time
                print("\n--- 20 SECOND WAIT START ---")
                self.next_state = States.WAIT_AFTER_CIRCLE

            case States.WAIT_AFTER_CIRCLE:
                now = self.get_clock().now().seconds_nanoseconds()[0]
                elapsed = now - self.wait_start_time
                remaining = int(20 - elapsed)

                if now - self.last_tick_time >= 1.0:
                    self.get_logger().info(f"Time remaining: {remaining}s...")
                    self.last_tick_time = now

                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    user_input = sys.stdin.readline().strip().lower()
                    if user_input == 'y':
                        self.next_state = States.IDLE
                        self.cycle_done = True

                if elapsed >= 20:
                    self.next_state = States.END

            case States.END:
                self.get_logger().warn("Shutdown.")
                rclpy.shutdown()

        if self.current_state != self.next_state:
            self.get_logger().info(f"TRANSITION: {self.current_state.name} -> {self.next_state.name}")
            self.current_state = self.next_state


def main(args=None):
    rclpy.init(args=args)
    node = MainStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()