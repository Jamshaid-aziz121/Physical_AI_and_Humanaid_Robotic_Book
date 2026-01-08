#!/usr/bin/env python3
"""
Joint Subscriber Example for the Physical AI & Humanoid Robotics Book

This node subscribes to joint states and logs the received data.
Demonstrates ROS 2 subscriber functionality for robot state monitoring.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    """
    Subscriber node that listens to joint states from humanoid robot
    """
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create subscription to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Topic where joint states are published
            self.joint_state_callback,
            10  # Queue size
        )

        # Also subscribe to the command topic to see what was sent
        self.command_subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # Prevent unused variable warnings
        self.subscription  # type: Optional[rclpy.Subscription]
        self.command_subscription  # type: Optional[rclpy.Subscription]

        self.get_logger().info('Joint State Subscriber node initialized')

    def joint_state_callback(self, msg: JointState):
        """
        Callback function to handle received joint state messages
        """
        self.get_logger().info(f'Received joint state update:')
        self.get_logger().info(f'  Joint names: {msg.name[:3]}...')  # Show first 3
        if msg.position:
            self.get_logger().info(f'  Positions: {[round(p, 3) for p in msg.position[:3]]}...')  # Show first 3
        if msg.velocity:
            self.get_logger().info(f'  Velocities: {[round(v, 3) for v in msg.velocity[:3]]}...')  # Show first 3
        if msg.effort:
            self.get_logger().info(f'  Efforts: {[round(e, 3) for e in msg.effort[:3]]}...')  # Show first 3

    def joint_command_callback(self, msg: JointState):
        """
        Callback function to handle received joint command messages
        """
        self.get_logger().info(f'Received joint command:')
        self.get_logger().info(f'  Commanded joints: {msg.name[:3]}...')  # Show first 3
        if msg.position:
            self.get_logger().info(f'  Commanded positions: {[round(p, 3) for p in msg.position[:3]]}...')  # Show first 3


def main(args=None):
    """
    Main function to initialize and run the joint state subscriber
    """
    rclpy.init(args=args)

    node = JointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()