#!/usr/bin/env python3
"""
Template for ROS 2 code examples in the Physical AI & Humanoid Robotics Book

This template follows the structure and conventions used throughout the book.
Replace the placeholder content with your specific implementation.
"""

import rclpy
from rclpy.node import Node

# Import required message types
from std_msgs.msg import String
# Add other imports as needed:
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Twist


class TemplateNode(Node):
    """
    Template class for ROS 2 nodes in the Physical AI & Humanoid Robotics Book
    """
    def __init__(self):
        # Initialize the node with a descriptive name
        super().__init__('template_node')

        # Create publishers (if needed)
        # self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create subscribers (if needed)
        # self.subscription = self.create_subscription(
        #     String,
        #     'topic_name',
        #     self.callback_function,
        #     10
        # )

        # Create timers (if needed)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # Declare parameters (if needed)
        # self.declare_parameter('param_name', 'default_value')

        # Log initialization
        self.get_logger().info('Template node initialized')


    def callback_function(self, msg):
        """
        Callback function for handling incoming messages
        """
        # Process the message
        self.get_logger().info(f'Received message: {msg.data}')


    def timer_callback(self):
        """
        Timer callback function for periodic tasks
        """
        # Create and publish message
        msg = String()
        msg.data = 'Hello from template node'
        # self.publisher.publish(msg)
        self.get_logger().info(f'Published message: {msg.data}')


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node instance
    node = TemplateNode()

    try:
        # Spin the node to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle shutdown gracefully
        node.get_logger().info('Interrupted by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()