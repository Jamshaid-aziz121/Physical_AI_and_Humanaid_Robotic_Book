#!/usr/bin/env python3
"""
Robot Control Service Example for the Physical AI & Humanoid Robotics Book

This node provides a service for controlling the humanoid robot.
Demonstrates ROS 2 service functionality for robot control commands.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger, SetBool
from std_msgs.msg import Bool


class RobotControlService(Node):
    """
    Service server that provides robot control functionality
    """
    def __init__(self):
        super().__init__('robot_control_service')

        # Create services
        self.start_service = self.create_service(
            Trigger,
            'start_robot_control',
            self.start_robot_callback
        )

        self.stop_service = self.create_service(
            Trigger,
            'stop_robot_control',
            self.stop_robot_callback
        )

        self.enable_motors_service = self.create_service(
            SetBool,
            'enable_motors',
            self.enable_motors_callback
        )

        # Publisher for motor enable status
        self.motor_status_pub = self.create_publisher(Bool, 'motor_enabled', 10)

        self.robot_running = False
        self.motors_enabled = False

        self.get_logger().info('Robot Control Service node initialized')

    def start_robot_callback(self, request, response):
        """
        Callback for starting robot control
        """
        self.get_logger().info('Received request to START robot control')

        # Simulate starting robot control
        self.robot_running = True

        response.success = True
        response.message = f'Robot control started. Running: {self.robot_running}'

        self.get_logger().info(f'Service response: {response.message}')
        return response

    def stop_robot_callback(self, request, response):
        """
        Callback for stopping robot control
        """
        self.get_logger().info('Received request to STOP robot control')

        # Simulate stopping robot control
        self.robot_running = False

        response.success = True
        response.message = f'Robot control stopped. Running: {self.robot_running}'

        self.get_logger().info(f'Service response: {response.message}')
        return response

    def enable_motors_callback(self, request, response):
        """
        Callback for enabling/disabling motors
        """
        enable = request.data
        self.get_logger().info(f'Received request to {"ENABLE" if enable else "DISABLE"} motors')

        # Simulate enabling/disabling motors
        self.motors_enabled = enable

        # Publish motor status
        status_msg = Bool()
        status_msg.data = self.motors_enabled
        self.motor_status_pub.publish(status_msg)

        response.success = True
        response.message = f'Motors {"enabled" if self.motors_enabled else "disabled"}'

        self.get_logger().info(f'Service response: {response.message}')
        return response


def main(args=None):
    """
    Main function to initialize and run the robot control service
    """
    rclpy.init(args=args)

    node = RobotControlService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()