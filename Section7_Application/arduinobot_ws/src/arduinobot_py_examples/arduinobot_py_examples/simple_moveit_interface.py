import rclpy  # Import ROS client library for Python
from rclpy.node import Node  # Import Node class from rclpy
from moveit_ros_planning_interface import MoveGroupInterface  # Import MoveGroupInterface for MoveIt integration
import sys  # Import system-specific parameters and functions

def move_robot(node):
    # Initialize MoveGroupInterface for the 'arm' and 'gripper' groups of the robot
    arm_move_group = MoveGroupInterface(node, "arm")
    gripper_move_group = MoveGroupInterface(node, "gripper")

    # Define joint goal positions for the arm and the gripper
    arm_joint_goal = [1.57, 0.0, 0.0]
    gripper_joint_goal = [-0.7, 0.7]

    # Set joint value targets for the arm and the gripper
    arm_within_bounds = arm_move_group.set_joint_value_target(arm_joint_goal)
    gripper_within_bounds = gripper_move_group.set_joint_value_target(gripper_joint_goal)

    # Check if the specified joint values are within the robot's limits
    if not arm_within_bounds or not gripper_within_bounds:
        node.get_logger().warn("Target joint position(s) were outside of limits, but we will plan and clamp to the limits")
        return  # Exit the function if any joint value is out of bounds

    # Plan the movements for both the arm and the gripper
    arm_plan = arm_move_group.plan()
    gripper_plan = gripper_move_group.plan()

    # Check if planning was successful for both the arm and the gripper
    if arm_plan and gripper_plan:
        node.get_logger().info("Planner SUCCEED, moving the arm and the gripper")
        # Execute the planned movements
        arm_move_group.go(wait=True)
        gripper_move_group.go(wait=True)
    else:
        # Log an error message if planning failed
        node.get_logger().error("One or more planners failed!")
        return  # Exit the function if planning failed

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS client library

    # Create a ROS node
    node = Node("simple_moveit_interface")
    try:
        move_robot(node)  # Call the function to move the robot
    except Exception as e:
        # Log any exceptions that occur
        node.get_logger().error('An exception occurred: {}'.format(str(e)))
        sys.exit(1)  # Exit the program on error

    rclpy.spin(node)  # Keep the node alive and responsive to callbacks
    rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == '__main__':
    main()  # Execute the main function if the script is run directly
