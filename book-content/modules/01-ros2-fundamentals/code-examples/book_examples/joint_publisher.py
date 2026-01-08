#!/usr/bin/env python3
"""
Joint Publisher Example for the Physical AI & Humanoid Robotics Book

This node publishes joint commands to control humanoid robot joints.
Demonstrates ROS 2 publisher functionality for robot control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from std_msgs.msg import Header


class JointCommandPublisher(Node):
    """
    Publisher node that sends joint commands for humanoid robot control
    """
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Create publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Timer to periodically publish joint commands
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for animation
        self.i = 0

        self.get_logger().info('Joint Command Publisher node initialized')

    def timer_callback(self):
        """
        Publish joint commands with oscillating positions
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Define joint names for humanoid robot
        joint_names = [
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        positions = []
        for idx, joint_name in enumerate(joint_names):
            # Create oscillating motion for each joint
            pos = 0.5 * math.sin(self.i * 0.05 + idx * 0.5)
            positions.append(pos)

        msg.name = joint_names
        msg.position = positions
        # Initialize empty velocity and effort arrays
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)

        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published joint commands: {dict(zip(joint_names[:3], positions[:3]))}')
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the joint command publisher
    """
    rclpy.init(args=args)

    node = JointCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()