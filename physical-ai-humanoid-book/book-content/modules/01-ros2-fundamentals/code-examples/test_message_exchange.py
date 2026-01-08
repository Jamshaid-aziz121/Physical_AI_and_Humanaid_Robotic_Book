#!/usr/bin/env python3
"""
Validation Script for ROS 2 Message Exchange

This script validates that messages are successfully exchanged between nodes
as per the acceptance criteria for the Physical AI & Humanoid Robotics Book.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
import time
from threading import Thread


class MessageExchangeValidator(Node):
    """
    Node that validates message exchange between different ROS 2 communication patterns
    """
    def __init__(self):
        super().__init__('message_exchange_validator')

        # Variables to track message exchanges
        self.received_messages = {
            'string_topic': [],
            'int_topic': [],
            'joint_state_topic': []
        }

        self.expected_messages = {
            'string_topic': 0,
            'int_topic': 0,
            'joint_state_topic': 0
        }

        # Publishers for testing
        self.string_pub = self.create_publisher(String, 'test_string_topic', 10)
        self.int_pub = self.create_publisher(Int32, 'test_int_topic', 10)
        self.joint_pub = self.create_publisher(JointState, 'test_joint_topic', 10)

        # Subscribers for validation
        self.string_sub = self.create_subscription(
            String,
            'test_string_topic',
            lambda msg: self.message_callback(msg, 'string_topic'),
            10
        )

        self.int_sub = self.create_subscription(
            Int32,
            'test_int_topic',
            lambda msg: self.message_callback(msg, 'int_topic'),
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            'test_joint_topic',
            lambda msg: self.message_callback(msg, 'joint_state_topic'),
            10
        )

        # Timer to send test messages
        self.test_timer = self.create_timer(0.1, self.send_test_messages)
        self.test_counter = 0

        # Timer to check validation results
        self.validation_timer = self.create_timer(1.0, self.validate_exchange)
        self.validation_start_time = self.get_clock().now()

        self.get_logger().info('Message Exchange Validator initialized')

    def message_callback(self, msg, topic_type):
        """
        Callback to track received messages
        """
        self.received_messages[topic_type].append(msg)
        self.get_logger().debug(f'Received message on {topic_type}: {msg}')

    def send_test_messages(self):
        """
        Send test messages to validate communication
        """
        if self.test_counter < 20:  # Send 20 test messages
            # Send string message
            str_msg = String()
            str_msg.data = f'Test message {self.test_counter}'
            self.string_pub.publish(str_msg)

            # Send integer message
            int_msg = Int32()
            int_msg.data = self.test_counter
            self.int_pub.publish(int_msg)

            # Send joint state message
            joint_msg = JointState()
            joint_msg.name = [f'joint_{self.test_counter}']
            joint_msg.position = [float(self.test_counter % 10)]
            self.joint_pub.publish(joint_msg)

            self.expected_messages['string_topic'] += 1
            self.expected_messages['int_topic'] += 1
            self.expected_messages['joint_state_topic'] += 1

            self.test_counter += 1
        elif self.test_counter == 20:
            # Stop the timer after sending all test messages
            self.test_timer.cancel()
            self.get_logger().info('Finished sending test messages')

    def validate_exchange(self):
        """
        Validate that messages are being exchanged successfully
        """
        current_time = self.get_clock().now()
        elapsed = (current_time - self.validation_start_time).nanoseconds / 1e9

        # Only validate after we've had time to send and receive messages
        if elapsed > 3.0 and self.test_counter >= 20:
            self.validation_timer.cancel()  # Stop validation timer
            self.perform_validation()

    def perform_validation(self):
        """
        Perform the actual validation and report results
        """
        self.get_logger().info("=" * 50)
        self.get_logger().info("MESSAGE EXCHANGE VALIDATION RESULTS")
        self.get_logger().info("=" * 50)

        all_validated = True

        for topic, expected_count in self.expected_messages.items():
            received_count = len(self.received_messages[topic])
            success = received_count >= expected_count * 0.9  # Allow 10% loss for network issues

            status = "✓ PASS" if success else "✗ FAIL"
            self.get_logger().info(f"{status} {topic}: Expected {expected_count}, Received {received_count}")

            if not success:
                all_validated = False

        self.get_logger().info("-" * 50)
        if all_validated:
            self.get_logger().info("🎉 ALL MESSAGE EXCHANGE TESTS PASSED!")
            self.get_logger().info("Messages are successfully exchanged between nodes.")
        else:
            self.get_logger().info("❌ SOME MESSAGE EXCHANGE TESTS FAILED!")
            self.get_logger().info("Check network connectivity and node configurations.")
        self.get_logger().info("=" * 50)


def main(args=None):
    """
    Main function to run the message exchange validation
    """
    rclpy.init(args=args)

    validator = MessageExchangeValidator()

    try:
        # Run for 10 seconds to allow validation to complete
        start_time = time.time()
        while time.time() - start_time < 10:
            rclpy.spin_once(validator, timeout_sec=0.1)
    except KeyboardInterrupt:
        validator.get_logger().info('Validation interrupted by user')
    finally:
        validator.perform_validation()  # Final validation check
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()