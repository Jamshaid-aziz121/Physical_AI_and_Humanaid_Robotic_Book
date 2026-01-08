#!/usr/bin/env python3
"""
Walk Action Server Example for the Physical AI & Humanoid Robotics Book

This node implements an action server for walking tasks.
Demonstrates ROS 2 action functionality for long-running robot tasks.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
import time

# Import the standard action messages for this example
# We'll use the example_interfaces/Fibonacci action as a placeholder
# In a real implementation, we would define our own Walk action
from example_interfaces.action import Fibonacci


class WalkActionServer(Node):
    """
    Action server that handles walking tasks for humanoid robot
    """
    def __init__(self):
        super().__init__('walk_action_server')

        # Create action server for walk tasks
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Using Fibonacci as example; in practice we'd define a Walk action
            'walk_forward',
            execute_callback=self.execute_callback,
            callback_group=None,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Walk Action Server node initialized')

    def goal_callback(self, goal_request):
        """
        Accept or reject a goal
        """
        self.get_logger().info('Received goal request')
        # Accept all goals for this example
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept or reject a cancellation request
        """
        self.get_logger().info('Received cancel request')
        # Accept all cancel requests for this example
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the goal and return result
        """
        self.get_logger().info('Executing goal...')

        # Simulate feedback and result for the walk action
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Simulate walking progress
        for i in range(1, goal_handle.request.order):
            # Check if there was a cancel request
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            # Update feedback
            feedback_msg.sequence.append(feedback_msg.sequence[-1] + feedback_msg.sequence[-2])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Simulate walking time
            time.sleep(0.5)

        # Goal succeeded
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Returning result: {result.sequence}')
        return result


def main(args=None):
    """
    Main function to initialize and run the walk action server
    """
    rclpy.init(args=args)

    node = WalkActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()