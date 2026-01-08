#!/usr/bin/env python3
"""
Humanoid Robot Controller for the Physical AI & Humanoid Robotics Book

This node implements a Python agent that controls simulated humanoid robot joints.
Demonstrates ROS 2 functionality for robot control using rclpy.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class HumanoidController(Node):
    """
    Python agent that controls simulated humanoid robot joints
    """
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publisher for joint trajectories
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Publisher for direct joint commands
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        # Subscriber to joint states (feedback)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer to send control commands
        timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        # Initialize controller variables
        self.time_counter = 0.0
        self.current_joint_states = {}

        self.get_logger().info('Humanoid Controller node initialized')

    def joint_state_callback(self, msg):
        """
        Callback to receive current joint states
        """
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]

    def control_loop(self):
        """
        Main control loop that sends joint commands
        """
        self.time_counter += 0.05  # Increment by timer period

        # Method 1: Send joint trajectory commands
        self.publish_joint_trajectory()

        # Method 2: Send direct joint state commands
        self.publish_joint_commands()

    def publish_joint_trajectory(self):
        """
        Publish joint trajectory commands for smooth motion
        """
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "base_link"

        # Define joint names for humanoid robot
        joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        traj_msg.joint_names = joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Calculate smooth oscillating trajectories for each joint
        positions = []
        velocities = []
        accelerations = []

        for i, joint_name in enumerate(joint_names):
            # Create smooth oscillating motion for each joint
            amplitude = 0.3
            frequency = 0.5
            phase_offset = i * 0.5

            pos = amplitude * math.sin(frequency * self.time_counter + phase_offset)
            vel = amplitude * frequency * math.cos(frequency * self.time_counter + phase_offset)
            acc = -amplitude * frequency * frequency * math.sin(frequency * self.time_counter + phase_offset)

            positions.append(pos)
            velocities.append(vel)
            accelerations.append(acc)

        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms

        traj_msg.points = [point]

        self.joint_traj_pub.publish(traj_msg)

    def publish_joint_commands(self):
        """
        Publish direct joint commands
        """
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "base_link"

        # Define joint names
        joint_names = [
            'head_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        cmd_msg.name = joint_names

        # Calculate positions for arm movements
        positions = []
        for i, joint_name in enumerate(joint_names):
            if joint_name == 'head_joint':
                # Head moves slowly
                pos = 0.1 * math.sin(0.2 * self.time_counter)
            else:
                # Arms move more actively
                pos = 0.4 * math.sin(0.7 * self.time_counter + i)
            positions.append(pos)

        cmd_msg.position = positions
        cmd_msg.velocity = [0.0] * len(positions)
        cmd_msg.effort = [0.0] * len(positions)

        self.joint_cmd_pub.publish(cmd_msg)

    def move_to_home_position(self):
        """
        Move all joints to home position
        """
        self.get_logger().info('Moving to home position')

        # Publish zero positions for all joints
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "base_link"

        joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'head_joint'
        ]

        cmd_msg.name = joint_names
        cmd_msg.position = [0.0] * len(joint_names)
        cmd_msg.velocity = [0.0] * len(joint_names)
        cmd_msg.effort = [0.0] * len(joint_names)

        self.joint_cmd_pub.publish(cmd_msg)


def main(args=None):
    """
    Main function to initialize and run the humanoid controller
    """
    rclpy.init(args=args)

    node = HumanoidController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        node.move_to_home_position()  # Move to safe position on exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()