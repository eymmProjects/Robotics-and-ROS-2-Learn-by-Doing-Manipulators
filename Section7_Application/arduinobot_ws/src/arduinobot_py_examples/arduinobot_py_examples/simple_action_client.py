import rclpy  # Import the ROS client library for Python
from rclpy.node import Node  # Import the Node class
from rclpy.action import ActionClient  # Import the ActionClient class
from arduinobot_msgs.action import Fibonacci  # Import the Fibonacci action from the custom action package

class SimpleActionClient(Node):  # Define a class that extends Node

    def __init__(self):
        super().__init__("simple_action_client")  # Initialize the Node with the name 'simple_action_client'
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")  # Create an ActionClient for the Fibonacci action
        self.goal = Fibonacci.Goal()  # Instantiate a Fibonacci Goal object
        self.goal.order = 10  # Set the Fibonacci sequence order to 10

        self.action_client.wait_for_server()  # Wait for the action server to be ready
        self.future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedbackCallback)  # Send the goal to the server asynchronously and specify a feedback callback
        self.future.add_done_callback(self.responseCallback)  # Add a callback for when the server responds to the goal request

    def responseCallback(self, future):
        goal_handle = future.result()  # Get the goal handle from the future object
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')  # Log if the goal was rejected
            return

        self.get_logger().info('Goal accepted :)')  # Log if the goal was accepted

        self.future = goal_handle.get_result_async()  # Request the result from the action server
        self.future.add_done_callback(self.resultCallback)  # Add a callback for when the result is available

    def resultCallback(self, future):
        result = future.result().result  # Get the result from the future object
        self.get_logger().info('Result: {0}'.format(result.sequence))  # Log the result (Fibonacci sequence)
        rclpy.shutdown()  # Shut down the ROS client library

    def feedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback  # Get the feedback from the feedback message
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))  # Log the received feedback (partial Fibonacci sequence)

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS client library
    action_client = SimpleActionClient()  # Instantiate the SimpleActionClient
    rclpy.spin(action_client)  # Keep the node alive and responsive

if __name__ == '__main__':
    main()  # Run the main function if the script is executed directly
