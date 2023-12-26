import rclpy  # Import the ROS2 client library (rclpy)
from rclpy.node import Node  # Import the Node class from rclpy.node module
from rclpy.action import ActionServer  # Import the ActionServer class from rclpy.action
from arduinobot_msgs.action import Fibonacci  # Import the Fibonacci action type from the arduinobot_msgs package
import time  # Import the time module for handling time-related tasks

# Define a class SimpleActionServer which inherits from Node
class SimpleActionServer(Node):
    def __init__(self):
        super().__init__("simple_action_server")  # Initialize the Node with the name 'simple_action_server'
        self.get_logger().info("Starting the Server")  # Log the message indicating the server is starting
        # Create an action server, specifying the action type (Fibonacci), and the callback function (goalCallback)
        self.action_server = ActionServer(
            self, Fibonacci, "fibonacci", self.goalCallback
        )

    # Define the goal callback function
    def goalCallback(self, goal_handle):
        # Log the reception of a goal request, including the order number from the request
        self.get_logger().info(
            "Received goal request with order %d" % goal_handle.request.order
        )

        # Create a feedback message object for the Fibonacci action
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]  # Initialize the Fibonacci sequence with the first two numbers

        # Calculate the Fibonacci sequence up to the requested order
        for i in range(1, goal_handle.request.order):
            # Append the next number in the sequence
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )
            # Log the current state of the Fibonacci sequence as feedback
            self.get_logger().info(
                "Feedback: {0}".format(feedback_msg.partial_sequence)
            )
            # Publish the feedback message
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # Sleep for 1 second to simulate work being done

        # Indicate that the action goal has been achieved
        goal_handle.succeed()

        # Prepare the result message with the final Fibonacci sequence
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result  # Return the result of the action

# Define the main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 client library
    simple_action_server = SimpleActionServer()  # Create an instance of the SimpleActionServer
    rclpy.spin(simple_action_server)  # Keep the server running, listening for new goals

# Entry point of the Python script
if __name__ == "__main__":
    main()  # Call the main function if the script is executed directly
