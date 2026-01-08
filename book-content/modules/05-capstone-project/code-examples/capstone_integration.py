#!/usr/bin/env python3
"""
Autonomous Humanoid Capstone Project - Complete System Integration

This script implements the capstone project that integrates all modules:
- ROS 2 fundamentals
- Physics simulation
- AI perception and navigation
- Vision-Language-Action integration
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from nav_msgs.msg import Odometry
import json
import time
import threading
from typing import Dict, Any, List, Optional
import math
import queue


class AutonomousHumanoidCapstone(Node):
    """
    Node that implements the complete autonomous humanoid system
    integrating all modules from the Physical AI & Humanoid Robotics Book
    """
    def __init__(self):
        super().__init__('autonomous_humanoid_capstone')

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.voice_text_pub = self.create_publisher(String, 'voice_text', 10)

        # Subscribers for sensor data and system status
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # Subscribers for system components
        self.intent_sub = self.create_subscription(String, 'recognized_intent', self.intent_callback, 10)
        self.action_status_sub = self.create_subscription(String, 'action_status', self.action_status_callback, 10)

        # Internal state
        self.current_pose = None
        self.laser_scan = None
        self.latest_image = None
        self.last_intent = None
        self.system_status = {}

        # Task queue for complex commands
        self.task_queue = queue.Queue()
        self.active_tasks = []

        # Navigation goals
        self.navigation_goals = {
            'kitchen': (2.0, 1.0, 0.0),
            'living_room': (0.0, 2.0, 1.57),
            'bedroom': (-2.0, 1.0, 3.14),
            'office': (-1.0, -2.0, -1.57),
            'home': (0.0, 0.0, 0.0),
            'charger': (0.0, 0.0, 0.0)
        }

        # Task planning and execution
        self.task_planner = TaskPlanner(self)
        self.behavior_manager = BehaviorManager(self)

        # System health monitoring
        self.system_health = {
            'ros_fundamentals_ok': True,
            'simulation_ok': True,
            'ai_navigation_ok': True,
            'vla_integration_ok': True,
            'overall_health': True
        }

        # Timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.system_monitor)

        self.get_logger().info('Autonomous Humanoid Capstone System initialized')

    def odom_callback(self, msg):
        """Callback for robot odometry"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Callback for laser scan data"""
        self.laser_scan = msg

    def image_callback(self, msg):
        """Callback for camera image data"""
        self.latest_image = msg

    def intent_callback(self, msg):
        """Callback for recognized intents"""
        try:
            intent_data = json.loads(msg.data)
            self.last_intent = intent_data

            # Process the intent if it's a complex command
            if intent_data.get('action') in ['GO_TO_LOCATION', 'FETCH_OBJECT', 'COMPLEX_TASK']:
                self.task_planner.process_complex_intent(intent_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in intent message')

    def action_status_callback(self, msg):
        """Callback for action status updates"""
        try:
            status_data = json.loads(msg.data)
            self.system_status.update(status_data)
        except json.JSONDecodeError:
            # If not JSON, treat as simple status string
            self.system_status['last_status'] = msg.data

    def system_monitor(self):
        """Monitor system health and performance"""
        # Check if all subsystems are responsive
        checks = [
            self.current_pose is not None,  # Localization working
            self.laser_scan is not None,    # Sensors working
            self.latest_image is not None,  # Vision working
            self.last_intent is not None    # VLA working
        ]

        self.system_health['overall_health'] = all(checks)

        if self.system_health['overall_health']:
            self.get_logger().debug('System health: All subsystems operational')
        else:
            self.get_logger().warning('System health: Some subsystems may be offline')

    def execute_complex_command(self, command: str):
        """
        Execute a complex command that may involve multiple steps
        Example: "Go to kitchen and bring red cup"
        """
        self.get_logger().info(f'Processing complex command: "{command}"')

        # Parse the command into subtasks
        subtasks = self.task_planner.parse_command(command)

        if not subtasks:
            self.get_logger().warning(f'Could not parse command: "{command}"')
            return False

        # Execute subtasks sequentially
        for task in subtasks:
            self.get_logger().info(f'Executing subtask: {task["action"]}')

            success = self.execute_single_task(task)
            if not success:
                self.get_logger().error(f'Task failed: {task["action"]}')
                return False

        self.get_logger().info('Complex command completed successfully')
        return True

    def execute_single_task(self, task: Dict[str, Any]) -> bool:
        """
        Execute a single task from the task plan
        """
        action = task['action']
        params = task.get('parameters', {})

        if action == 'NAVIGATE_TO':
            return self.navigate_to_location(params.get('location'))
        elif action == 'DETECT_OBJECT':
            return self.detect_object(params.get('object_name'))
        elif action == 'GRAB_OBJECT':
            return self.grab_object(params.get('object_name'))
        elif action == 'RETURN_WITH_OBJECT':
            return self.return_to_origin()
        else:
            self.get_logger().warning(f'Unknown task action: {action}')
            return False

    def navigate_to_location(self, location: str) -> bool:
        """Navigate to a specific location"""
        if location not in self.navigation_goals:
            self.get_logger().error(f'Unknown location: {location}')
            return False

        target_x, target_y, target_theta = self.navigation_goals[location]
        self.get_logger().info(f'Navigating to {location} at ({target_x}, {target_y})')

        # In a real system, this would use navigation stack
        # For simulation, we'll just move toward the target
        return self.simulate_navigation_to(target_x, target_y)

    def detect_object(self, object_name: str) -> bool:
        """Detect a specific object using vision system"""
        self.get_logger().info(f'Detecting object: {object_name}')

        # In a real system, this would process camera images
        # For simulation, we'll just return success
        time.sleep(1.0)  # Simulate processing time

        # Simulate detection success/failure based on object visibility
        detection_success = True  # Simulated success

        if detection_success:
            self.get_logger().info(f'Object {object_name} detected successfully')
        else:
            self.get_logger().warning(f'Object {object_name} not found')

        return detection_success

    def grab_object(self, object_name: str) -> bool:
        """Attempt to grab an object"""
        self.get_logger().info(f'Attempting to grab object: {object_name}')

        # In a real system, this would control manipulator
        # For simulation, we'll just wait
        time.sleep(2.0)  # Simulate grabbing time

        grab_success = True  # Simulated success

        if grab_success:
            self.get_logger().info(f'Successfully grabbed {object_name}')
        else:
            self.get_logger().error(f'Failed to grab {object_name}')

        return grab_success

    def return_to_origin(self) -> bool:
        """Return to starting location"""
        self.get_logger().info('Returning to origin')
        return self.simulate_navigation_to(0.0, 0.0)

    def simulate_navigation_to(self, target_x: float, target_y: float) -> bool:
        """
        Simulate navigation to a target position
        In a real system, this would use the navigation stack
        """
        if not self.current_pose:
            self.get_logger().error('Cannot navigate without current pose')
            return False

        # Calculate distance to target
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        # Simple proportional controller simulation
        if distance > 0.2:  # 20cm tolerance
            # Calculate direction to target
            angle_to_target = math.atan2(target_y - current_y, target_x - current_x)

            # Move toward target
            twist_msg = Twist()
            twist_msg.linear.x = min(distance * 0.5, 0.5)  # Max 0.5 m/s
            twist_msg.angular.z = (angle_to_target - self.get_yaw_from_quaternion()) * 0.5

            # Publish command
            self.cmd_vel_pub.publish(twist_msg)

            # Wait a bit for movement
            time.sleep(0.5)

            # Stop briefly
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.1)

            # Recursively call until close enough
            return self.simulate_navigation_to(target_x, target_y)
        else:
            # Close enough to target
            self.cmd_vel_pub.publish(Twist())  # Stop
            self.get_logger().info(f'Reached destination ({target_x}, {target_y})')
            return True

    def get_yaw_from_quaternion(self) -> float:
        """Extract yaw from quaternion orientation"""
        if not self.current_pose:
            return 0.0

        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def run_capstone_demo(self):
        """
        Run the capstone demonstration with complex commands
        """
        demo_commands = [
            "Go to kitchen and bring red cup",
            "Navigate to bedroom and take a picture",
            "Return to home position",
            "Wave to the person"
        ]

        self.get_logger().info('Starting capstone demonstration...')

        for i, command in enumerate(demo_commands):
            self.get_logger().info(f'Demo step {i+1}/{len(demo_commands)}: {command}')

            # Send command as voice input
            voice_msg = String()
            voice_msg.data = command
            self.voice_text_pub.publish(voice_msg)

            # Execute the complex command
            success = self.execute_complex_command(command)

            if success:
                self.get_logger().info(f'✓ Command "{command}" completed successfully')
            else:
                self.get_logger().error(f'✗ Command "{command}" failed')

            # Wait between commands
            time.sleep(3.0)

        self.get_logger().info('Capstone demonstration completed!')


class TaskPlanner:
    """
    Component responsible for parsing complex commands and creating task plans
    """
    def __init__(self, capstone_node: AutonomousHumanoidCapstone):
        self.capstone = capstone_node

    def parse_command(self, command: str) -> List[Dict[str, Any]]:
        """
        Parse a natural language command into a sequence of tasks
        """
        command_lower = command.lower()
        tasks = []

        # Handle "go to X and bring Y" command
        if 'and bring' in command_lower or 'and get' in command_lower:
            # Split command into navigation and object retrieval parts
            parts = command_lower.split(' and ')
            if len(parts) >= 2:
                # Extract location from first part
                location = self.extract_location(parts[0])

                # Extract object from second part
                object_name = self.extract_object(parts[1])

                if location and object_name:
                    tasks.extend([
                        {'action': 'NAVIGATE_TO', 'parameters': {'location': location}},
                        {'action': 'DETECT_OBJECT', 'parameters': {'object_name': object_name}},
                        {'action': 'GRAB_OBJECT', 'parameters': {'object_name': object_name}},
                        {'action': 'RETURN_WITH_OBJECT', 'parameters': {}}
                    ])

        # Handle simple navigation commands
        elif 'go to' in command_lower or 'navigate to' in command_lower:
            location = self.extract_location(command_lower)
            if location:
                tasks.append({'action': 'NAVIGATE_TO', 'parameters': {'location': location}})

        # Handle object-related commands
        elif 'bring' in command_lower or 'get' in command_lower or 'fetch' in command_lower:
            object_name = self.extract_object(command_lower)
            if object_name:
                tasks.extend([
                    {'action': 'DETECT_OBJECT', 'parameters': {'object_name': object_name}},
                    {'action': 'GRAB_OBJECT', 'parameters': {'object_name': object_name}}
                ])

        return tasks

    def extract_location(self, text: str) -> Optional[str]:
        """
        Extract location from text
        """
        # Look for known locations in the text
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'home', 'charger']

        for loc in locations:
            if loc in text:
                return loc.replace(' ', '_')  # Convert to internal format

        return None

    def extract_object(self, text: str) -> Optional[str]:
        """
        Extract object name from text
        """
        # Look for object descriptors in the text
        # This is simplified - in reality would use NLP
        object_words = []

        # Common object descriptors
        descriptors = ['cup', 'bottle', 'box', 'ball', 'toy', 'book', 'phone', 'keys']

        for desc in descriptors:
            if desc in text:
                # Look for color descriptors nearby
                colors = ['red', 'blue', 'green', 'yellow', 'black', 'white']
                for color in colors:
                    if color in text:
                        object_words.append(f'{color} {desc}')
                        break
                else:
                    object_words.append(desc)
                break

        return object_words[0] if object_words else None

    def process_complex_intent(self, intent_data: Dict[str, Any]):
        """
        Process a complex intent that may require task planning
        """
        action = intent_data.get('action', '').upper()

        if action in ['GO_TO_LOCATION', 'FETCH_OBJECT', 'COMPLEX_TASK']:
            # Add to task queue for execution
            self.capstone.task_queue.put(intent_data)


class BehaviorManager:
    """
    Component responsible for managing robot behaviors
    """
    def __init__(self, capstone_node: AutonomousHumanoidCapstone):
        self.capstone = capstone_node

    def execute_behavior(self, behavior_name: str, parameters: Dict[str, Any] = None) -> bool:
        """
        Execute a specific robot behavior
        """
        if behavior_name == 'wave':
            return self.execute_wave_behavior(parameters)
        elif behavior_name == 'dance':
            return self.execute_dance_behavior(parameters)
        elif behavior_name == 'greet':
            return self.execute_greet_behavior(parameters)
        else:
            self.capstone.get_logger().warning(f'Unknown behavior: {behavior_name}')
            return False

    def execute_wave_behavior(self, parameters: Dict[str, Any] = None):
        """Execute waving behavior"""
        self.capstone.get_logger().info('Executing waving behavior')
        # In a real system, this would control arm joints
        # For simulation, just log
        time.sleep(2.0)
        return True

    def execute_dance_behavior(self, parameters: Dict[str, Any] = None):
        """Execute dancing behavior"""
        self.capstone.get_logger().info('Executing dancing behavior')
        # In a real system, this would execute dance sequence
        # For simulation, just log
        time.sleep(4.0)
        return True

    def execute_greet_behavior(self, parameters: Dict[str, Any] = None):
        """Execute greeting behavior"""
        self.capstone.get_logger().info('Executing greeting behavior')
        # In a real system, this might combine movement and speech
        # For simulation, just log
        time.sleep(1.5)
        return True


def main(args=None):
    """
    Main function to run the autonomous humanoid capstone project
    """
    rclpy.init(args=args)

    capstone = AutonomousHumanoidCapstone()

    # Create multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(capstone)

    print("Autonomous Humanoid Capstone System")
    print("=====================================")
    print("Integrating all modules from the Physical AI & Humanoid Robotics Book:")
    print("- ROS 2 fundamentals")
    print("- Physics simulation")
    print("- AI perception and navigation")
    print("- Vision-Language-Action integration")
    print("")
    print("System initialized. Starting capstone demonstration...")

    try:
        # Run the capstone demonstration
        capstone.run_capstone_demo()

        # Keep running for ongoing operation
        print("\nCapstone system running. Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        capstone.get_logger().info('Capstone system interrupted by user')
    finally:
        capstone.destroy_node()
        rclpy.shutdown()

    print("Autonomous Humanoid Capstone System shutdown complete.")


if __name__ == '__main__':
    main()