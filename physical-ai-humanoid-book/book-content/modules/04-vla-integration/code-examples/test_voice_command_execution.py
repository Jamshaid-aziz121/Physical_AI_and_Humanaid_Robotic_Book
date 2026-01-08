#!/usr/bin/env python3
"""
Voice Command Validation Script

This script validates that voice commands generate appropriate ROS actions
and execute correctly by the robot, fulfilling the acceptance criteria.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import json
import time
from threading import Event
from typing import Dict, Any, List
import math


class VoiceCommandValidator(Node):
    """
    Node that validates voice command execution by the robot
    """
    def __init__(self):
        super().__init__('voice_command_validator')

        # Publishers for sending test commands
        self.voice_text_pub = self.create_publisher(String, 'voice_text', 10)
        self.command_pub = self.create_publisher(String, 'robot_command', 10)

        # Subscribers for monitoring robot state
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.intent_sub = self.create_subscription(String, 'recognized_intent', self.intent_callback, 10)
        self.action_status_sub = self.create_subscription(String, 'action_status', self.action_status_callback, 10)

        # Robot state tracking
        self.current_pose = None
        self.last_cmd_vel = None
        self.last_intent = None
        self.action_status = None

        # Validation tracking
        self.command_responses = []
        self.movement_records = []
        self.intent_records = []

        # Synchronization
        self.response_received = Event()
        self.intent_received = Event()
        self.movement_detected = Event()

        # Test parameters
        self.test_commands = [
            "move forward",
            "turn left",
            "stop",
            "go to kitchen",
            "wave",
            "take picture"
        ]

        self.test_results = {
            'total_tests': 0,
            'passed_tests': 0,
            'failed_tests': 0,
            'test_details': []
        }

        # Timer for validation
        self.test_timer = self.create_timer(5.0, self.run_validation_cycle)

        self.get_logger().info('Voice Command Validator initialized')

    def odom_callback(self, msg):
        """Callback to receive robot odometry"""
        self.current_pose = msg.pose.pose

    def cmd_vel_callback(self, msg):
        """Callback to receive robot velocity commands"""
        self.last_cmd_vel = msg
        self.movement_detected.set()

        # Record movement
        movement_record = {
            'timestamp': time.time(),
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z
        }
        self.movement_records.append(movement_record)

    def intent_callback(self, msg):
        """Callback to receive recognized intents"""
        try:
            intent_data = json.loads(msg.data)
            self.last_intent = intent_data
            self.intent_received.set()

            # Record intent
            intent_record = {
                'timestamp': time.time(),
                'intent': intent_data
            }
            self.intent_records.append(intent_record)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in intent message')

    def action_status_callback(self, msg):
        """Callback to receive action status updates"""
        self.action_status = msg.data

    def send_test_command(self, command: str):
        """Send a test voice command to the system"""
        self.get_logger().info(f'Sending test command: "{command}"')

        # Send voice text command
        voice_msg = String()
        voice_msg.data = command
        self.voice_text_pub.publish(voice_msg)

        # Also send as robot command
        cmd_msg = String()
        cmd_msg.data = json.dumps({
            'action': 'GENERAL_COMMAND',
            'command_text': command,
            'request_id': int(time.time() * 1000)
        })
        self.command_pub.publish(cmd_msg)

    def run_validation_cycle(self):
        """Run a validation cycle with test commands"""
        if not hasattr(self, 'current_test_index'):
            self.current_test_index = 0

        if self.current_test_index < len(self.test_commands):
            command = self.test_commands[self.current_test_index]

            # Reset events
            self.response_received.clear()
            self.intent_received.clear()
            self.movement_detected.clear()

            # Send command
            self.send_test_command(command)

            # Wait for response with timeout
            intent_success = self.intent_received.wait(timeout=3.0)
            movement_success = self.movement_detected.wait(timeout=5.0)

            # Validate results
            test_result = self.validate_command_execution(command, intent_success, movement_success)
            self.test_results['test_details'].append(test_result)

            if test_result['passed']:
                self.test_results['passed_tests'] += 1
            else:
                self.test_results['failed_tests'] += 1

            self.test_results['total_tests'] += 1

            self.current_test_index += 1
        else:
            # All tests completed, generate report
            self.generate_validation_report()
            self.current_test_index = 0  # Reset for next cycle

    def validate_command_execution(self, command: str, intent_success: bool, movement_success: bool) -> Dict[str, Any]:
        """Validate execution of a single command"""
        result = {
            'command': command,
            'intent_recognized': intent_success,
            'movement_executed': movement_success,
            'passed': False,
            'details': []
        }

        # Check if intent was recognized
        if intent_success and self.last_intent:
            recognized_action = self.last_intent.get('action', '').lower()
            result['details'].append(f'Recognized action: {recognized_action}')

            # Validate that the recognized action matches expectation
            expected_actions = {
                'move forward': ['move_forward', 'move'],
                'turn left': ['turn_left', 'rotate'],
                'stop': ['stop', 'halt'],
                'go to kitchen': ['go_to_location', 'move_to'],
                'wave': ['wave'],
                'take picture': ['take_picture', 'capture']
            }

            command_lower = command.lower()
            if command_lower in expected_actions:
                expected_keywords = expected_actions[command_lower]
                if any(keyword in recognized_action for keyword in expected_keywords):
                    result['intent_correct'] = True
                    result['details'].append('Correct intent recognized')
                else:
                    result['intent_correct'] = False
                    result['details'].append(f'Expected action containing {expected_keywords}, got {recognized_action}')
            else:
                result['intent_correct'] = True  # Unknown command, just check that something was recognized
        else:
            result['intent_correct'] = False
            result['details'].append('No intent recognized')

        # Check if movement was executed
        if movement_success and self.last_cmd_vel:
            linear_magnitude = math.sqrt(
                self.last_cmd_vel.linear.x**2 +
                self.last_cmd_vel.linear.y**2 +
                self.last_cmd_vel.linear.z**2
            )
            angular_magnitude = math.sqrt(
                self.last_cmd_vel.angular.x**2 +
                self.last_cmd_vel.angular.y**2 +
                self.last_cmd_vel.angular.z**2
            )

            if linear_magnitude > 0.01 or angular_magnitude > 0.01:
                result['movement_valid'] = True
                result['details'].append(f'Movement executed (linear: {linear_magnitude:.3f}, angular: {angular_magnitude:.3f})')
            else:
                result['movement_valid'] = False
                result['details'].append('Movement command sent but no significant motion detected')
        else:
            result['movement_valid'] = False
            result['details'].append('No movement command detected')

        # Determine overall pass/fail
        result['passed'] = result['intent_correct'] and result['movement_valid']

        return result

    def generate_validation_report(self):
        """Generate comprehensive validation report"""
        self.get_logger().info("=" * 80)
        self.get_logger().info("VOICE COMMAND EXECUTION VALIDATION REPORT")
        self.get_logger().info("=" * 80)

        # Summary statistics
        if self.test_results['total_tests'] > 0:
            success_rate = (self.test_results['passed_tests'] / self.test_results['total_tests']) * 100
            success_threshold = 80.0  # 80% success rate required

            self.get_logger().info(f"Overall Statistics:")
            self.get_logger().info(f"  Total Tests: {self.test_results['total_tests']}")
            self.get_logger().info(f"  Passed: {self.test_results['passed_tests']}")
            self.get_logger().info(f"  Failed: {self.test_results['failed_tests']}")
            self.get_logger().info(f"  Success Rate: {success_rate:.1f}%")
            self.get_logger().info(f"  Required: {success_threshold}%")
            self.get_logger().info(f"  Status: {'✓ PASS' if success_rate >= success_threshold else '✗ FAIL'}")
        else:
            self.get_logger().info("No tests executed yet")
            return

        self.get_logger().info("\nDetailed Results:")

        for i, test_detail in enumerate(self.test_results['test_details']):
            self.get_logger().info(f"\n  Test {i+1}: '{test_detail['command']}'")
            self.get_logger().info(f"    Intent Recognized: {'✓' if test_detail['intent_recognized'] else '✗'}")
            self.get_logger().info(f"    Movement Executed: {'✓' if test_detail['movement_executed'] else '✗'}")
            self.get_logger().info(f"    Correct Intent: {'✓' if test_detail.get('intent_correct', False) else '✗'}")
            self.get_logger().info(f"    Valid Movement: {'✓' if test_detail.get('movement_valid', False) else '✗'}")
            self.get_logger().info(f"    Result: {'✓ PASS' if test_detail['passed'] else '✗ FAIL'}")

            for detail in test_detail['details']:
                self.get_logger().info(f"    - {detail}")

        # Additional metrics
        self.get_logger().info(f"\nAdditional Metrics:")
        self.get_logger().info(f"  Total Intent Records: {len(self.intent_records)}")
        self.get_logger().info(f"  Total Movement Records: {len(self.movement_records)}")
        self.get_logger().info(f"  Current Pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})" if self.current_pose else "  Current Pose: Unknown")

        # Overall validation result
        overall_pass = success_rate >= success_threshold
        self.get_logger().info("-" * 80)

        if overall_pass:
            self.get_logger().info("🎉 VOICE COMMAND EXECUTION VALIDATION PASSED!")
            self.get_logger().info("Voice commands successfully generate appropriate ROS actions and execute by robot.")
            self.get_logger().info(f"Success rate: {success_rate:.1f}% (>{success_threshold}% required)")
        else:
            self.get_logger().info("❌ VOICE COMMAND EXECUTION VALIDATION FAILED!")
            self.get_logger().info("Voice commands do not consistently generate appropriate ROS actions.")
            self.get_logger().info(f"Success rate: {success_rate:.1f}% (<{success_threshold}% required)")

        self.get_logger().info("=" * 80)

    def validate_single_command(self, command: str, timeout: float = 10.0) -> bool:
        """
        Validate execution of a single command with specified timeout
        """
        self.get_logger().info(f"Validating single command: '{command}'")

        # Reset tracking
        self.response_received.clear()
        self.intent_received.clear()
        self.movement_detected.clear()

        # Send command
        self.send_test_command(command)

        # Wait for responses
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.intent_received.is_set() and self.movement_detected.is_set():
                break
            time.sleep(0.1)

        # Validate
        intent_success = self.intent_received.is_set()
        movement_success = self.movement_detected.is_set()

        result = self.validate_command_execution(command, intent_success, movement_success)

        self.get_logger().info(f"Single command validation result: {'PASS' if result['passed'] else 'FAIL'}")

        return result['passed']


def main(args=None):
    """
    Main function to run the voice command validation
    """
    rclpy.init(args=args)

    validator = VoiceCommandValidator()

    try:
        # Run validation for 60 seconds to allow multiple test cycles
        start_time = time.time()
        while time.time() - start_time < 60:
            rclpy.spin_once(validator, timeout_sec=0.1)
    except KeyboardInterrupt:
        validator.get_logger().info('Voice Command Validation interrupted by user')
    finally:
        validator.generate_validation_report()
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()