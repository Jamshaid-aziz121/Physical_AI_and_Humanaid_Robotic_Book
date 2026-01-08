#!/usr/bin/env python3
"""
Robot Control Client Example for the Physical AI & Humanoid Robotics Book

This node demonstrates calling robot control services.
Demonstrates ROS 2 service client functionality for robot control.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger, SetBool
from std_msgs.msg import Bool


class RobotControlClient(Node):
    """
    Service client that calls robot control services
    """
    def __init__(self):
        super().__init__('robot_control_client')

        # Create service clients
        self.start_cli = self.create_client(Trigger, 'start_robot_control')
        self.stop_cli = self.create_client(Trigger, 'stop_robot_control')
        self.enable_motors_cli = self.create_client(SetBool, 'enable_motors')

        # Subscriber to monitor motor status
        self.motor_status_sub = self.create_subscription(
            Bool,
            'motor_enabled',
            self.motor_status_callback,
            10
        )

        # Wait for services to be available
        while not self.start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Start service not available, waiting again...')

        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stop service not available, waiting again...')

        while not self.enable_motors_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Enable motors service not available, waiting again...')

        # Timer to periodically call services
        timer_period = 5.0  # Call services every 5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.call_count = 0
        self.motors_enabled = False

        self.get_logger().info('Robot Control Client node initialized')

    def motor_status_callback(self, msg: Bool):
        """
        Callback for monitoring motor status
        """
        self.motors_enabled = msg.data
        self.get_logger().info(f'Motor status updated: {"ENABLED" if self.motors_enabled else "DISABLED"}')

    def timer_callback(self):
        """
        Periodically call robot control services
        """
        self.get_logger().info(f'Calling robot control services (call #{self.call_count})')

        # Alternate between starting and stopping robot control
        if self.call_count % 4 == 0:
            self.call_start_robot()
        elif self.call_count % 4 == 1:
            self.call_enable_motors(True)
        elif self.call_count % 4 == 2:
            self.call_stop_robot()
        else:
            self.call_enable_motors(False)

        self.call_count += 1

    def call_start_robot(self):
        """
        Call the start robot control service
        """
        request = Trigger.Request()
        future = self.start_cli.call_async(request)
        future.add_done_callback(self.start_response_callback)

    def call_stop_robot(self):
        """
        Call the stop robot control service
        """
        request = Trigger.Request()
        future = self.stop_cli.call_async(request)
        future.add_done_callback(self.stop_response_callback)

    def call_enable_motors(self, enable: bool):
        """
        Call the enable motors service
        """
        request = SetBool.Request()
        request.data = enable
        future = self.enable_motors_cli.call_async(request)
        future.add_done_callback(self.enable_motors_response_callback)

    def start_response_callback(self, future):
        """
        Callback for start robot service response
        """
        try:
            response = future.result()
            self.get_logger().info(f'Start robot response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def stop_response_callback(self, future):
        """
        Callback for stop robot service response
        """
        try:
            response = future.result()
            self.get_logger().info(f'Stop robot response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def enable_motors_response_callback(self, future):
        """
        Callback for enable motors service response
        """
        try:
            response = future.result()
            self.get_logger().info(f'Enable motors response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    """
    Main function to initialize and run the robot control client
    """
    rclpy.init(args=args)

    node = RobotControlClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()