#!/usr/bin/env python3
"""
Walk Action Client Example for the Physical AI & Humanoid Robotics Book

This node demonstrates calling the walk action server.
Demonstrates ROS 2 action client functionality for long-running robot tasks.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

# Using Fibonacci as example; in practice we'd define a Walk action
from example_interfaces.action import Fibonacci


class WalkActionClient(Node):
    """
    Action client that calls the walk action server
    """
    def __init__(self):
        super().__init__('walk_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'walk_forward'
        )

        # Timer to send goals periodically
        self.timer = self.create_timer(10.0, self.send_goal)
        self.goal_count = 0

        self.get_logger().info('Walk Action Client node initialized')

    def send_goal(self):
        """
        Send a goal to the action server
        """
        self.get_logger().info(f'Sending goal #{self.goal_count}...')

        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            return

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10  # Example: walk 10 steps

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.goal_count += 1

    def goal_response_callback(self, future):
        """
        Callback for when the goal is accepted or rejected
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for receiving feedback
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence[-3:]}...')  # Show last 3 elements

    def get_result_callback(self, future):
        """
        Callback for when the result is received
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        self.get_logger().info('Goal completed')


def main(args=None):
    """
    Main function to initialize and run the walk action client
    """
    rclpy.init(args=args)

    node = WalkActionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()