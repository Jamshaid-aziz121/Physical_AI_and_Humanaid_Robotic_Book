#!/usr/bin/env python3
"""
VLA Integration Module - Complete System Test

This script performs an end-to-end test of the Vision-Language-Action integration system,
validating that all components work together correctly.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
import json
import time
from threading import Event
from typing import Dict, Any, List
import subprocess
import os


class VLAIntegrationTester(Node):
    """
    Node that performs end-to-end testing of the VLA integration system
    """
    def __init__(self):
        super().__init__('vla_integration_tester')

        # Publishers for sending test commands
        self.voice_text_pub = self.create_publisher(String, 'voice_text', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        # Subscribers for monitoring system responses
        self.intent_sub = self.create_subscription(String, 'recognized_intent', self.intent_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.action_status_sub = self.create_subscription(String, 'action_status', self.action_status_callback, 10)

        # State tracking
        self.received_intents = []
        self.received_movements = []
        self.action_statuses = []

        # Test synchronization
        self.intent_received = Event()
        self.movement_received = Event()
        self.test_results = {
            'components_initialized': False,
            'voice_recognition_working': False,
            'intent_processing_working': False,
            'action_execution_working': False,
            'overall_system_health': False
        }

        # Start component health check
        self.timer = self.create_timer(1.0, self.health_check)

        self.get_logger().info('VLA Integration Tester initialized')

    def intent_callback(self, msg: String):
        """Callback for received intents"""
        try:
            intent_data = json.loads(msg.data)
            self.received_intents.append(intent_data)
            self.intent_received.set()
            self.get_logger().info(f'Received intent: {intent_data["action"]}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in intent message')

    def cmd_vel_callback(self, msg: Twist):
        """Callback for received movement commands"""
        movement_data = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z,
            'timestamp': time.time()
        }
        self.received_movements.append(movement_data)
        self.movement_received.set()

    def action_status_callback(self, msg: String):
        """Callback for action status updates"""
        self.action_statuses.append(msg.data)

    def health_check(self):
        """Check the health of all system components"""
        # Check if we have any activity
        if len(self.received_intents) > 0 or len(self.received_movements) > 0:
            self.test_results['components_initialized'] = True

    def send_voice_command(self, command: str):
        """Send a voice command to the system"""
        msg = String()
        msg.data = command
        self.voice_text_pub.publish(msg)
        self.get_logger().info(f'Sent voice command: "{command}"')

    def run_complete_system_test(self):
        """Run a complete system test"""
        self.get_logger().info('Starting VLA Integration System Test...')

        # Test sequence
        test_commands = [
            "move forward",
            "turn left",
            "stop",
            "go to kitchen",
            "wave",
            "take picture"
        ]

        results = {
            'total_commands_sent': len(test_commands),
            'commands_processed': 0,
            'intents_recognized': 0,
            'movements_executed': 0,
            'success_rate': 0.0
        }

        for i, command in enumerate(test_commands):
            self.get_logger().info(f'Test {i+1}/{len(test_commands)}: Sending command "{command}"')

            # Clear events
            self.intent_received.clear()
            self.movement_received.clear()

            # Send command
            self.send_voice_command(command)

            # Wait for responses
            intent_success = self.intent_received.wait(timeout=5.0)
            movement_success = self.movement_received.wait(timeout=5.0)

            if intent_success:
                results['intents_recognized'] += 1
            if movement_success:
                results['movements_executed'] += 1

            results['commands_processed'] += 1

            self.get_logger().info(f'  Intent recognized: {intent_success}, Movement executed: {movement_success}')

            # Small delay between commands
            time.sleep(2.0)

        # Calculate success rate
        if results['commands_processed'] > 0:
            results['success_rate'] = (results['intents_recognized'] + results['movements_executed']) / (results['commands_processed'] * 2)

        self.get_logger().info(f'VLA System Test Results:')
        self.get_logger().info(f'  Commands sent: {results["total_commands_sent"]}')
        self.get_logger().info(f'  Commands processed: {results["commands_processed"]}')
        self.get_logger().info(f'  Intents recognized: {results["intents_recognized"]}')
        self.get_logger().info(f'  Movements executed: {results["movements_executed"]}')
        self.get_logger().info(f'  Success rate: {results["success_rate"]:.2f}')

        # Update overall test results
        self.test_results['voice_recognition_working'] = results['intents_recognized'] > 0
        self.test_results['action_execution_working'] = results['movements_executed'] > 0
        self.test_results['overall_system_health'] = results['success_rate'] > 0.5

        return results

    def generate_comprehensive_report(self):
        """Generate a comprehensive test report"""
        self.get_logger().info("=" * 80)
        self.get_logger().info("VLA INTEGRATION SYSTEM - COMPREHENSIVE TEST REPORT")
        self.get_logger().info("=" * 80)

        # Component status
        self.get_logger().info("Component Status:")
        self.get_logger().info(f"  Components Initialized: {'✓' if self.test_results['components_initialized'] else '✗'}")
        self.get_logger().info(f"  Voice Recognition Working: {'✓' if self.test_results['voice_recognition_working'] else '✗'}")
        self.get_logger().info(f"  Intent Processing Working: {'✓' if self.test_results['intent_processing_working'] else '✗'}")
        self.get_logger().info(f"  Action Execution Working: {'✓' if self.test_results['action_execution_working'] else '✗'}")
        self.get_logger().info(f"  Overall System Health: {'✓' if self.test_results['overall_system_health'] else '✗'}")

        # Data statistics
        self.get_logger().info(f"\nData Statistics:")
        self.get_logger().info(f"  Total Intents Received: {len(self.received_intents)}")
        self.get_logger().info(f"  Total Movements Received: {len(self.received_movements)}")
        self.get_logger().info(f"  Total Action Status Updates: {len(self.action_statuses)}")

        # Sample recent intents
        if self.received_intents:
            self.get_logger().info(f"\nRecent Intents:")
            for intent in self.received_intents[-5:]:  # Last 5 intents
                self.get_logger().info(f"  - {intent.get('action', 'unknown')}: {intent.get('confidence', 0.0):.2f}")

        # Overall assessment
        working_components = sum([
            self.test_results['voice_recognition_working'],
            self.test_results['action_execution_working'],
            self.test_results['overall_system_health']
        ])

        total_components = 3

        self.get_logger().info(f"\nOverall Assessment:")
        self.get_logger().info(f"  Working Components: {working_components}/{total_components}")
        self.get_logger().info(f"  Status: {'🎉 SYSTEM INTEGRATION SUCCESSFUL' if working_components == total_components else '⚠️  PARTIAL SUCCESS'}")

        self.get_logger().info("=" * 80)

        return working_components == total_components


def main(args=None):
    """
    Main function to run the VLA integration test
    """
    rclpy.init(args=args)

    tester = VLAIntegrationTester()

    try:
        # Wait a bit for system initialization
        time.sleep(2.0)

        # Run the complete system test
        test_results = tester.run_complete_system_test()

        # Wait a bit more to collect all responses
        time.sleep(5.0)

        # Generate comprehensive report
        success = tester.generate_comprehensive_report()

        # Print final outcome
        if success:
            print("\n✅ VLA Integration Module - All Systems Operational")
            print("The Vision-Language-Action integration system is functioning correctly.")
        else:
            print("\n❌ VLA Integration Module - Some Issues Detected")
            print("Please check the component status and resolve any issues.")

    except KeyboardInterrupt:
        tester.get_logger().info('VLA Integration Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()