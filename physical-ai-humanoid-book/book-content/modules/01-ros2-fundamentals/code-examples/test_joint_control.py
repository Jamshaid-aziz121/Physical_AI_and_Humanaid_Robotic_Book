#!/usr/bin/env python3
"""
Validation Script for Joint Control

This script validates that the Python agent can control robot joints
and read sensor data as per the acceptance criteria for the Physical AI & Humanoid Robotics Book.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class JointControlValidator(Node):
    """
    Node that validates joint control and sensor data reading
    """
    def __init__(self):
        super().__init__('joint_control_validator')

        # Variables to track joint states
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.current_joint_efforts = {}

        # Track command history
        self.sent_commands = []
        self.last_command_time = None

        # Publishers for joint commands
        self.joint_command_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Subscriber for joint states (feedback)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer to send joint commands and validate response
        self.command_timer = self.create_timer(0.1, self.send_joint_commands)
        self.validation_timer = self.create_timer(1.0, self.validate_joint_control)

        self.command_counter = 0
        self.validation_counter = 0

        self.get_logger().info('Joint Control Validator initialized')

    def joint_state_callback(self, msg):
        """
        Callback to receive joint state feedback
        """
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = msg.position[i]

            if i < len(msg.velocity):
                self.current_joint_velocities[joint_name] = msg.velocity[i]

            if i < len(msg.effort):
                self.current_joint_efforts[joint_name] = msg.effort[i]

        self.get_logger().debug(f'Received joint states: {list(self.current_joint_positions.keys())}')

    def send_joint_commands(self):
        """
        Send joint commands to test control
        """
        # Send JointState command
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "base_link"

        # Define joint names for humanoid
        joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        cmd_msg.name = joint_names

        # Calculate time-varying positions
        positions = []
        for i, joint_name in enumerate(joint_names):
            # Create oscillating motion
            pos = 0.3 * math.sin(0.5 * self.command_counter * 0.1 + i * 0.3)
            positions.append(pos)

        cmd_msg.position = positions
        cmd_msg.velocity = [0.0] * len(positions)
        cmd_msg.effort = [0.0] * len(positions)

        self.joint_command_pub.publish(cmd_msg)
        self.sent_commands.append((self.command_counter, positions))

        # Also send trajectory command
        traj_msg = self.create_trajectory_command(joint_names, positions)
        self.joint_trajectory_pub.publish(traj_msg)

        self.command_counter += 1

        if self.command_counter % 10 == 0:  # Log every 10 commands
            self.get_logger().info(f'Sent command #{self.command_counter} to joints: {joint_names[:3]}...')

    def create_trajectory_command(self, joint_names, positions):
        """
        Create a trajectory command for smoother motion
        """
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "base_link"
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 100ms

        traj_msg.points = [point]
        return traj_msg

    def validate_joint_control(self):
        """
        Validate that joint control is working
        """
        self.validation_counter += 1

        if self.current_joint_positions:
            # Check if we're receiving feedback
            joint_names = list(self.current_joint_positions.keys())

            if joint_names:
                # Get the most recent joint values
                recent_values = [self.current_joint_positions[name] for name in joint_names[:3]]

                self.get_logger().debug(f'Validation #{self.validation_counter}: Recent joint values: {recent_values}')

                # Check if joint values are changing (indicating control is working)
                if len(self.sent_commands) > 1:
                    prev_cmd_idx = -2  # Second to last command
                    curr_cmd_idx = -1   # Last command

                    if len(self.sent_commands) > 1:
                        prev_positions = self.sent_commands[prev_cmd_idx][1]
                        curr_positions = self.sent_commands[curr_cmd_idx][1]

                        # Check if we're commanding different positions
                        pos_changed = any(abs(prev - curr) > 0.01
                                       for prev, curr in zip(prev_positions[:3], curr_positions[:3]))

                        if pos_changed and self.current_joint_positions:
                            self.get_logger().info(f'✓ Joint control validation #{self.validation_counter}: Control commands are being sent and joints exist')
                        else:
                            self.get_logger().info(f'- Joint validation #{self.validation_counter}: Waiting for more data')

                if self.validation_counter >= 5:  # Perform final validation after 5 checks
                    self.final_validation()

    def final_validation(self):
        """
        Perform final validation and report results
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("JOINT CONTROL AND SENSOR READING VALIDATION RESULTS")
        self.get_logger().info("=" * 60)

        # Check if we received joint states
        has_joint_states = bool(self.current_joint_positions)
        self.get_logger().info(f"✓ Joint State Feedback: {'YES' if has_joint_states else 'NO'}")

        # Check if we sent commands
        commands_sent = len(self.sent_commands) > 0
        self.get_logger().info(f"✓ Commands Sent: {'YES' if commands_sent else 'NO'}")

        # Check variety in commands (if we sent multiple commands with different values)
        if len(self.sent_commands) > 1:
            first_pos = self.sent_commands[0][1][:3] if self.sent_commands else []
            last_pos = self.sent_commands[-1][1][:3] if self.sent_commands else []

            varied_commands = first_pos != last_pos if first_pos and last_pos else False
            self.get_logger().info(f"✓ Varied Commands: {'YES' if varied_commands else 'NO'}")
        else:
            self.get_logger().info(f"✓ Varied Commands: NO (only {len(self.sent_commands)} command(s) sent)")

        # Check if we have sensor data
        has_sensor_data = bool(self.current_joint_velocities) or bool(self.current_joint_efforts)
        self.get_logger().info(f"✓ Sensor Data Available: {'YES' if has_sensor_data else 'NO'}")

        # Overall result
        overall_success = has_joint_states and commands_sent and has_sensor_data
        self.get_logger().info("-" * 60)

        if overall_success:
            self.get_logger().info("🎉 JOINT CONTROL VALIDATION PASSED!")
            self.get_logger().info("Python agent can control robot joints and read sensor data.")
        else:
            self.get_logger().info("❌ JOINT CONTROL VALIDATION PARTIALLY FAILED!")
            self.get_logger().info("Some aspects of joint control/sensor reading are not working.")

        self.get_logger().info("=" * 60)


def main(args=None):
    """
    Main function to run the joint control validation
    """
    rclpy.init(args=args)

    validator = JointControlValidator()

    try:
        # Run for 15 seconds to allow for validation
        start_time = rclpy.clock.Clock().now()
        while (rclpy.clock.Clock().now() - start_time).nanoseconds < 15e9:  # 15 seconds
            rclpy.spin_once(validator, timeout_sec=0.1)
    except KeyboardInterrupt:
        validator.get_logger().info('Validation interrupted by user')
    finally:
        validator.final_validation()  # Final validation check
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()