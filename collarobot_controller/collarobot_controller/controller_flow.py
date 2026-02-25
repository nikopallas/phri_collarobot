import rclpy
from rclpy.node import Node

import time
import sys
import select
from rclpy.action import ActionClient
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from enum import Enum
import json
from typing import Dict, List, Optional


# Import the model models to get the main_brain feedback
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


class VisionNodeSubscriber(Node):
    def __init__(self):
        super().__init__('collarobot_vision_node_subscriber')
        self.received_msg = None
        self.subscription = self.create_subscription(
            String, 'input_topic', self.listener_callback, 10)
        self.latest_data = None
        self.previous_data = None  # Store previous state for comparison

    def listener_callback(self, msg):
        self.latest_data = msg.data
        self.get_logger().info(f'got messag from Vision: {msg.data}')

        # Check if status changed when new data arrives
        if self.is_ingredient_status_changed():
            self.get_logger().info('New numbers added to proposed or accepted zones!')

    def is_ingredient_status_changed(self):
        """
        Checks if new numbers have been added to 'proposed' or 'accepted' zones.
        Returns True if new items are detected, False otherwise.
        """
        if not self.latest_data:
            return False

        try:
            # Parse current data
            current_data = json.loads(self.latest_data)

            # If this is the first message, store it and return False
            if self.previous_data is None:
                self.previous_data = current_data
                return False

            # Check for new numbers in 'proposed' zone
            proposed_changed = self._has_new_numbers(
                self.previous_data.get('proposed', []),
                current_data.get('proposed', [])
            )

            # Check for new numbers in 'accepted' zone
            accepted_changed = self._has_new_numbers(
                self.previous_data.get('accepted', []),
                current_data.get('accepted', [])
            )

            # Update previous data for next comparison
            self.previous_data = current_data

            return proposed_changed or accepted_changed

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error checking status change: {e}')
            return False

    def _has_new_numbers(self, previous_list: List[int], current_list: List[int]) -> bool:
        """
        Helper method to check if new numbers have been added to a list.
        Returns True if current list has numbers that weren't in previous list.
        """
        # Convert to sets for easier comparison
        previous_set = set(previous_list)
        current_set = set(current_list)

        # Check if there are any numbers in current that weren't in previous
        new_numbers = current_set - previous_set

        if new_numbers:
            self.get_logger().info(f'New numbers detected: {new_numbers}')
            return True

        return False



class MainStateMachineNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.current_state = States.IDLE
        self.next_state = States.IDLE
        self.cycle_done = True
        self.wait_start_time = 0.0
        self.last_tick_time = 0.0
        self.vision_subscriber = VisionNodeSubscriber()
        # self.action_client = ActionClient()
        # self.add_node_to_executor(self.vision_subscriber)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Node started with models.py integration.")

    def subscribe_from_vision(self) :
        has_new_ingredient = self.vision_subscriber.is_ingredient_status_changed()
        return has_new_ingredient

    def send_motion_robot_controller(self, motion_name):
        self.get_logger().info(f">>> SENDING MOTION: {motion_name}")
        return # TEMP
        # 1. Check server availability
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        # 2. Wrap the string in the actual Goal message type
        # Replace 'MyActionType' with your actual Action name (e.g., RobotMotion)
        """
         my actionType sould be adjusted with the action name 
         action name will be defined later in action file
        """
        goal_msg = MyActionType.Goal()
        goal_msg.motion_name = motion_name  # Ensure 'motion_name' exists in your .action file

        # 3. Send Async (DO NOT sleep after this)
        self.get_logger().info(f"Goal '{motion_name}' is being dispatched...")

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # Optional: if you want live updates
        )

        # This callback handles the "Accepted/Rejected" response automatically
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Retrieve the motion name using the future as a key
        motion_name = self._pending_goals.pop(future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Motion '{motion_name}' REJECTED by server.")
            return

        self.get_logger().info(f"Motion '{motion_name}' ACCEPTED. Execution starting...")

        # To keep track of it until it's FINISHED, we pass it to the next future
        get_result_future = goal_handle.get_result_async()

        # We use a 'lambda' here to "sneak" the motion_name into the next function
        get_result_future.add_done_callback(
            lambda f: self.get_result_callback(f, motion_name)
        )

    def get_result_callback(self, future, motion_name):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f" FINISHED: The '{motion_name}' action is complete.")

        else:
            self.get_logger().warn(f" FAILED: The '{motion_name}' action ended with status: {status}")

    def timer_callback(self):
        match self.current_state:
            case States.IDLE:
                if self.cycle_done:
                    self.cycle_done = False
                    self.next_state = States.HAND_INVITATION

            case States.HAND_INVITATION:
                self.send_motion_robot_controller("Hand Invitation")
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
                self.send_motion_robot_controller("Propose New")
                self.next_state = States.MAIN_BRAIN

            case States.PUT_INGREDIENT_BACK:
                self.send_motion_robot_controller("Put Back")
                self.next_state = States.MAIN_BRAIN

            case States.ACCEPT_INGREDIENT:
                self.send_motion_robot_controller("Accept")
                self.next_state = States.MAIN_BRAIN

            case States.HAND_CIRCLE:
                self.send_motion_robot_controller("Hand Circle")
                self.wait_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.last_tick_time = self.wait_start_time
                self.get_logger().info("\n--- 20 SECOND WAIT START ---")
                #print("\n--- 20 SECOND WAIT START ---")
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