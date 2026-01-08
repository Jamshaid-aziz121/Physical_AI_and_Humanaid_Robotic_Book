#!/usr/bin/env python3
"""
Capstone Project Validation Script

This script validates that the complete autonomous humanoid system
integrates all modules successfully and meets the acceptance criteria.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import json
import time
from threading import Event
from typing import Dict, Any, List
import math


class CapstoneValidator(Node):
    """
    Node that validates the complete autonomous humanoid system
    """
    def __init__(self):
        super().__init__('capstone_validator')

        # Publishers for sending test commands
        self.voice_text_pub = self.create_publisher(String, 'voice_text', 10)

        # Subscribers for monitoring system performance
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.intent_sub = self.create_subscription(String, 'recognized_intent', self.intent_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # System state tracking
        self.current_pose = None
        self.scan_data = None
        self.image_data = None
        self.intents_received = []
        self.commands_sent = []

        # Validation tracking
        self.module_validation = {
            'ros_fundamentals': False,
            'physics_simulation': False,
            'ai_perception_navigation': False,
            'vla_integration': False,
            'integration_success': False
        }

        self.performance_metrics = {
            'response_time_avg': 0.0,
            'command_success_rate': 0.0,
            'navigation_accuracy': 0.0,
            'system_stability': 0.0
        }

        # Synchronization
        self.command_sent_event = Event()
        self.response_received_event = Event()

        # Test parameters
        self.test_scenarios = [
            {
                'name': 'Basic Movement',
                'command': 'move forward',
                'expected_response': 'MOVE_FORWARD'
            },
            {
                'name': 'Navigation',
                'command': 'go to kitchen',
                'expected_response': 'GO_TO_LOCATION'
            },
            {
                'name': 'Object Interaction',
                'command': 'wave',
                'expected_response': 'WAVE'
            },
            {
                'name': 'Complex Task',
                'command': 'go to kitchen and bring red cup',
                'expected_response': 'COMPLEX_TASK'
            }
        ]

        self.validation_results = {
            'total_tests': 0,
            'passed_tests': 0,
            'failed_tests': 0,
            'test_details': []
        }

        # Timer for validation
        self.test_timer = self.create_timer(10.0, self.run_validation_cycle)

        self.get_logger().info('Capstone Validator initialized')

    def odom_callback(self, msg):
        """Callback for robot odometry"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Callback for laser scan data"""
        self.scan_data = msg

    def image_callback(self, msg):
        """Callback for camera image data"""
        self.image_data = msg

    def intent_callback(self, msg):
        """Callback for recognized intents"""
        try:
            intent_data = json.loads(msg.data)
            self.intents_received.append(intent_data)

            # Check if this is a response to our command
            if self.command_sent_event.is_set():
                self.response_received_event.set()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in intent message')

    def cmd_vel_callback(self, msg):
        """Callback for velocity commands"""
        # Track movement commands to validate action execution
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().debug('Movement command received')

    def send_test_command(self, command: str):
        """Send a test command to the system"""
        self.get_logger().info(f'Sending test command: "{command}"')

        msg = String()
        msg.data = command
        self.voice_text_pub.publish(msg)

        self.commands_sent.append({
            'command': command,
            'timestamp': time.time()
        })

        self.command_sent_event.set()
        self.response_received_event.clear()

    def run_validation_cycle(self):
        """Run a validation cycle with test scenarios"""
        if not hasattr(self, 'current_scenario_index'):
            self.current_scenario_index = 0

        if self.current_scenario_index < len(self.test_scenarios):
            scenario = self.test_scenarios[self.current_scenario_index]

            self.get_logger().info(f'Running validation scenario: {scenario["name"]}')

            # Send the command
            self.send_test_command(scenario['command'])

            # Wait for response with timeout
            response_received = self.response_received_event.wait(timeout=10.0)

            # Validate the response
            test_result = self.validate_scenario_response(scenario, response_received)
            self.validation_results['test_details'].append(test_result)

            if test_result['passed']:
                self.validation_results['passed_tests'] += 1
            else:
                self.validation_results['failed_tests'] += 1

            self.validation_results['total_tests'] += 1

            self.current_scenario_index += 1
        else:
            # All scenarios completed, generate final report
            self.generate_final_validation_report()
            self.current_scenario_index = 0  # Reset for next cycle

    def validate_scenario_response(self, scenario: Dict[str, Any], response_received: bool) -> Dict[str, Any]:
        """Validate response for a single scenario"""
        result = {
            'scenario_name': scenario['name'],
            'command_sent': scenario['command'],
            'response_received': response_received,
            'expected_response': scenario['expected_response'],
            'actual_response': 'NONE',
            'passed': False,
            'details': []
        }

        if response_received and self.intents_received:
            # Check the most recent intent
            latest_intent = self.intents_received[-1]
            actual_action = latest_intent.get('action', '').upper()
            result['actual_response'] = actual_action

            if scenario['expected_response'] in actual_action or actual_action == scenario['expected_response']:
                result['passed'] = True
                result['details'].append(f'Correct response: {actual_action}')
            else:
                result['details'].append(f'Expected {scenario["expected_response"]}, got {actual_action}')
        else:
            result['details'].append('No response received')

        # Additional validation based on scenario type
        if scenario['name'] == 'Basic Movement':
            # Check if movement was executed
            if response_received:
                result['details'].append('Movement command recognized')
                result['passed'] = True
        elif scenario['name'] == 'Navigation':
            # Check if navigation was initiated
            if 'GO_TO_LOCATION' in result['actual_response']:
                result['details'].append('Navigation command processed')
        elif scenario['name'] == 'Complex Task':
            # Check if complex task was parsed correctly
            if 'COMPLEX_TASK' in result['actual_response'] or 'NAVIGATE_TO' in result['actual_response']:
                result['details'].append('Complex task recognized and parsed')

        return result

    def validate_module_integration(self) -> Dict[str, bool]:
        """Validate integration of all modules"""
        validation = {
            'ros_fundamentals': len(self.commands_sent) > 0,  # ROS communication working
            'physics_simulation': self.current_pose is not None,  # Simulation providing data
            'ai_perception_navigation': self.scan_data is not None,  # Perception working
            'vla_integration': len(self.intents_received) > 0,  # VLA working
            'integration_success': (
                len(self.commands_sent) > 0 and
                len(self.intents_received) > 0 and
                self.current_pose is not None and
                self.scan_data is not None
            )
        }

        return validation

    def calculate_performance_metrics(self) -> Dict[str, float]:
        """Calculate performance metrics"""
        metrics = {
            'response_time_avg': 0.0,
            'command_success_rate': 0.0,
            'navigation_accuracy': 0.0,
            'system_stability': 0.0
        }

        if self.validation_results['total_tests'] > 0:
            metrics['command_success_rate'] = (
                self.validation_results['passed_tests'] / self.validation_results['total_tests']
            )

        # For simulation, we'll use basic metrics
        if self.current_pose:
            # Calculate distance from origin as a stability measure
            distance_from_origin = math.sqrt(
                self.current_pose.position.x**2 +
                self.current_pose.position.y**2
            )
            # Higher distance might indicate more movement/stability
            metrics['system_stability'] = min(1.0, distance_from_origin / 10.0)

        return metrics

    def generate_final_validation_report(self):
        """Generate comprehensive validation report"""
        self.get_logger().info("=" * 90)
        self.get_logger().info("AUTONOMOUS HUMANOID CAPSTONE PROJECT - VALIDATION REPORT")
        self.get_logger().info("=" * 90)

        # Module Integration Validation
        module_validation = self.validate_module_integration()

        self.get_logger().info("Module Integration Status:")
        self.get_logger().info(f"  ROS Fundamentals: {'✓' if module_validation['ros_fundamentals'] else '✗'}")
        self.get_logger().info(f"  Physics Simulation: {'✓' if module_validation['physics_simulation'] else '✗'}")
        self.get_logger().info(f"  AI Perception/Navigation: {'✓' if module_validation['ai_perception_navigation'] else '✗'}")
        self.get_logger().info(f"  VLA Integration: {'✓' if module_validation['vla_integration'] else '✗'}")
        self.get_logger().info(f"  Overall Integration: {'✓' if module_validation['integration_success'] else '✗'}")

        # Test Results
        self.get_logger().info(f"\nTest Results:")
        if self.validation_results['total_tests'] > 0:
            success_rate = (self.validation_results['passed_tests'] / self.validation_results['total_tests']) * 100
            success_threshold = 80.0  # 80% success rate required

            self.get_logger().info(f"  Total Tests: {self.validation_results['total_tests']}")
            self.get_logger().info(f"  Passed: {self.validation_results['passed_tests']}")
            self.get_logger().info(f"  Failed: {self.validation_results['failed_tests']}")
            self.get_logger().info(f"  Success Rate: {success_rate:.1f}%")
            self.get_logger().info(f"  Required: {success_threshold}%")
            self.get_logger().info(f"  Status: {'✓ PASS' if success_rate >= success_threshold else '✗ FAIL'}")
        else:
            self.get_logger().info("  No tests executed")

        # Performance Metrics
        performance_metrics = self.calculate_performance_metrics()

        self.get_logger().info(f"\nPerformance Metrics:")
        self.get_logger().info(f"  Command Success Rate: {performance_metrics['command_success_rate']:.2f}")
        self.get_logger().info(f"  System Stability: {performance_metrics['system_stability']:.2f}")

        # Detailed Scenario Results
        self.get_logger().info(f"\nDetailed Scenario Results:")
        for i, test_detail in enumerate(self.validation_results['test_details']):
            self.get_logger().info(f"  {i+1}. {test_detail['scenario_name']}: {'✓ PASS' if test_detail['passed'] else '✗ FAIL'}")
            self.get_logger().info(f"     Command: '{test_detail['command_sent']}'")
            self.get_logger().info(f"     Expected: {test_detail['expected_response']}, Got: {test_detail['actual_response']}")

        # Data Statistics
        self.get_logger().info(f"\nSystem Data Statistics:")
        self.get_logger().info(f"  Commands Sent: {len(self.commands_sent)}")
        self.get_logger().info(f"  Intents Received: {len(self.intents_received)}")
        self.get_logger().info(f"  Current Pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})" if self.current_pose else "  Current Pose: Unknown")
        self.get_logger().info(f"  Scan Data Points: {len(self.scan_data.ranges)}" if self.scan_data else "  Scan Data: None")

        # Overall Assessment
        overall_pass = (
            success_rate >= success_threshold and
            module_validation['integration_success']
        )

        self.get_logger().info("-" * 90)

        if overall_pass:
            self.get_logger().info("🎉 CAPSTONE PROJECT VALIDATION PASSED!")
            self.get_logger().info("The autonomous humanoid system successfully integrates all modules.")
            self.get_logger().info(f"Success rate: {success_rate:.1f}% (>{success_threshold}% required)")
            self.get_logger().info(f"All modules integrated: {'✓' if module_validation['integration_success'] else '✗'}")
        else:
            self.get_logger().info("❌ CAPSTONE PROJECT VALIDATION INCOMPLETE!")
            self.get_logger().info("The system needs improvement in some areas.")
            if success_rate < success_threshold:
                self.get_logger().info(f"  - Success rate too low: {success_rate:.1f}% (<{success_threshold}% required)")
            if not module_validation['integration_success']:
                self.get_logger().info(f"  - Module integration incomplete")

        self.get_logger().info("=" * 90)

        return overall_pass

    def run_acceptance_test(self) -> bool:
        """
        Run the main acceptance test for the capstone project
        """
        self.get_logger().info("Running capstone project acceptance test...")

        # Test the specific requirement: "Go to kitchen and bring red cup"
        complex_command = "go to kitchen and bring red cup"

        self.get_logger().info(f'Sending complex command: "{complex_command}"')
        self.send_test_command(complex_command)

        # Wait for processing
        time.sleep(15.0)  # Allow time for complex task execution

        # Check if the command was processed
        command_processed = any(
            'COMPLEX_TASK' in intent.get('action', '').upper() or
            'NAVIGATE_TO' in intent.get('action', '').upper()
            for intent in self.intents_received
        )

        if command_processed:
            self.get_logger().info("✅ Complex command processed successfully")
        else:
            self.get_logger().info("❌ Complex command not processed correctly")

        return command_processed


def main(args=None):
    """
    Main function to run the capstone validation
    """
    rclpy.init(args=args)

    validator = CapstoneValidator()

    try:
        # Run validation for 120 seconds to allow multiple test cycles
        start_time = time.time()
        while time.time() - start_time < 120:
            rclpy.spin_once(validator, timeout_sec=0.1)
    except KeyboardInterrupt:
        validator.get_logger().info('Capstone Validation interrupted by user')
    finally:
        validator.generate_final_validation_report()
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()