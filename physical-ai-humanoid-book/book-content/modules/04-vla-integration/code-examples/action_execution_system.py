#!/usr/bin/env python3
"""
Action Execution System for Robotics Command Processing

This module implements the action execution layer that translates high-level
robot commands into specific ROS action calls and manages their execution.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json
import time
import math
from typing import Dict, Any, Optional, List, Tuple
from enum import Enum
from threading import Lock
from collections import deque
import asyncio


class ActionResult(Enum):
    """
    Result of action execution
    """
    SUCCESS = "success"
    FAILURE = "failure"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"
    IN_PROGRESS = "in_progress"


class ActionPriority(Enum):
    """
    Priority levels for actions
    """
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4


class ActionExecutionSystem(Node):
    """
    System that manages execution of robot actions with priority and coordination
    """
    def __init__(self):
        super().__init__('action_execution_system')

        # Publishers for basic commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.status_pub = self.create_publisher(String, 'action_status', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10
        )
        self.intent_sub = self.create_subscription(
            String,
            'structured_robot_command',
            self.intent_callback,
            10
        )
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Action clients for complex operations
        self.navigation_client = ActionClient(
            self,
            'nav2_msgs/action/NavigateToPose',
            'navigate_to_pose',
            callback_group=ReentrantCallbackGroup()
        )

        self.manipulation_client = ActionClient(
            self,
            'control_msgs/action/FollowJointTrajectory',
            'manipulator_controller/follow_joint_trajectory',
            callback_group=ReentrantCallbackGroup()
        )

        # Internal state
        self.current_pose = None
        self.action_queue = deque()
        self.active_actions = {}
        self.action_history = []
        self.executor = MultiThreadedExecutor(num_threads=4)

        # Configuration parameters
        self.declare_parameter('default_linear_speed', 0.5)
        self.declare_parameter('default_angular_speed', 0.5)
        self.declare_parameter('default_movement_duration', 2.0)
        self.declare_parameter('action_timeout', 30.0)
        self.declare_parameter('max_queue_size', 10)
        self.declare_parameter('concurrent_actions_limit', 3)

        self.default_linear_speed = self.get_parameter('default_linear_speed').value
        self.default_angular_speed = self.get_parameter('default_angular_speed').value
        self.default_movement_duration = self.get_parameter('default_movement_duration').value
        self.action_timeout = self.get_parameter('action_timeout').value
        self.max_queue_size = self.get_parameter('max_queue_size').value
        self.concurrent_actions_limit = self.get_parameter('concurrent_actions_limit').value

        # Threading and synchronization
        self.queue_lock = Lock()
        self.action_lock = Lock()

        # Timer for periodic action processing
        self.action_timer = self.create_timer(0.1, self.process_action_queue)

        self.get_logger().info('Action Execution System initialized')

    def odom_callback(self, msg: Odometry):
        """
        Callback to track robot pose
        """
        self.current_pose = msg.pose.pose

    def command_callback(self, msg: String):
        """
        Callback to receive robot commands
        """
        try:
            command_data = json.loads(msg.data)

            # Parse command structure
            if isinstance(command_data, str):
                # Simple command string
                action_request = {
                    'action': command_data,
                    'priority': ActionPriority.NORMAL.value,
                    'parameters': {},
                    'source': 'command_topic'
                }
            else:
                # Structured command
                action_request = {
                    'action': command_data.get('action', 'unknown'),
                    'priority': command_data.get('priority', ActionPriority.NORMAL.value),
                    'parameters': command_data.get('parameters', {}),
                    'source': 'command_topic',
                    'request_id': command_data.get('request_id', int(time.time() * 1000))
                }

            self.queue_action(action_request)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def intent_callback(self, msg: String):
        """
        Callback to receive structured robot commands from intent recognizer
        """
        try:
            intent_data = json.loads(msg.data)

            action_request = {
                'action': intent_data.get('action', 'unknown'),
                'priority': ActionPriority.NORMAL.value,
                'parameters': intent_data.get('parameters', {}),
                'source': 'intent_recognizer',
                'confidence': intent_data.get('confidence', 0.0),
                'original_text': intent_data.get('raw_text', ''),
                'request_id': int(time.time() * 1000)
            }

            # Only accept high-confidence intents
            if action_request.get('confidence', 0.0) >= 0.7:
                self.queue_action(action_request)
            else:
                self.get_logger().warning(f'Low confidence intent ignored: {action_request["confidence"]}')

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON intent: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing intent: {e}')

    def queue_action(self, action_request: Dict[str, Any]):
        """
        Queue an action for execution with priority management
        """
        with self.queue_lock:
            # Check queue size limit
            if len(self.action_queue) >= self.max_queue_size:
                self.get_logger().warning('Action queue is full, dropping oldest action')
                dropped_action = self.action_queue.popleft()
                self.get_logger().info(f'Dropped action: {dropped_action.get("action", "unknown")}')

            # Add action to queue
            self.action_queue.append(action_request)

            # Sort queue by priority (higher priority first)
            self.action_queue = deque(sorted(
                self.action_queue,
                key=lambda x: x.get('priority', ActionPriority.NORMAL.value),
                reverse=True
            ))

            self.get_logger().info(f'Queued action: {action_request["action"]} (Priority: {action_request["priority"]})')

    def process_action_queue(self):
        """
        Process actions from the queue based on availability and priority
        """
        with self.queue_lock:
            if not self.action_queue:
                return

            # Check concurrent action limit
            active_count = len([a for a in self.active_actions.values()
                              if a['status'] == ActionResult.IN_PROGRESS])

            if active_count >= self.concurrent_actions_limit:
                return  # Wait for some actions to complete

            # Take next action from queue
            action_request = self.action_queue.popleft()

        # Execute the action
        self.execute_action_async(action_request)

    def execute_action_async(self, action_request: Dict[str, Any]):
        """
        Execute an action asynchronously
        """
        action_id = f"action_{action_request.get('request_id', int(time.time() * 1000))}"

        # Track action
        with self.action_lock:
            self.active_actions[action_id] = {
                'request': action_request,
                'status': ActionResult.IN_PROGRESS,
                'start_time': time.time(),
                'result': None
            }

        # Execute in a separate thread/task
        self.executor.create_task(
            lambda: self._execute_action_worker(action_id, action_request)
        )

    def _execute_action_worker(self, action_id: str, action_request: Dict[str, Any]):
        """
        Worker function to execute an action
        """
        try:
            action_type = action_request['action']
            parameters = action_request['parameters']

            self.get_logger().info(f'Executing action {action_id}: {action_type}')

            # Execute the specific action
            success = self.execute_specific_action(action_type, parameters)

            # Update action status
            with self.action_lock:
                if action_id in self.active_actions:
                    self.active_actions[action_id]['status'] = ActionResult.SUCCESS if success else ActionResult.FAILURE
                    self.active_actions[action_id]['result'] = success

            # Log action completion
            self.log_action_completion(action_id, success)

        except Exception as e:
            self.get_logger().error(f'Action {action_id} failed: {e}')
            with self.action_lock:
                if action_id in self.active_actions:
                    self.active_actions[action_id]['status'] = ActionResult.FAILURE
                    self.active_actions[action_id]['result'] = str(e)

    def execute_specific_action(self, action_type: str, parameters: Dict[str, Any]) -> bool:
        """
        Execute a specific action type with given parameters
        """
        action_handlers = {
            'MOVE_FORWARD': self.handle_move_forward,
            'MOVE_BACKWARD': self.handle_move_backward,
            'TURN_LEFT': self.handle_turn_left,
            'TURN_RIGHT': self.handle_turn_right,
            'STOP': self.handle_stop,
            'PICK_UP': self.handle_pick_up,
            'PUT_DOWN': self.handle_put_down,
            'WAVE': self.handle_wave,
            'DANCE': self.handle_dance,
            'FOLLOW_ME': self.handle_follow_me,
            'GREET_USER': self.handle_greet_user,
            'TAKE_PICTURE': self.handle_take_picture,
            'GO_TO_LOCATION': self.handle_go_to_location,
            'LOOK_AT': self.handle_look_at,
            'GRAB_OBJECT': self.handle_grab_object,
            'RELEASE_OBJECT': self.handle_release_object,
            'WAIT': self.handle_wait,
            'RETURN_HOME': self.handle_return_home,
            'GENERAL_COMMAND': self.handle_general_command
        }

        handler = action_handlers.get(action_type.upper())
        if handler:
            return handler(parameters)
        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')
            return False

    def handle_move_forward(self, parameters: Dict[str, Any]) -> bool:
        """Handle move forward action"""
        duration = parameters.get('duration', self.default_movement_duration)
        speed = parameters.get('speed', self.default_linear_speed)

        twist_msg = Twist()
        twist_msg.linear.x = speed

        self.get_logger().info(f'Moving forward for {duration}s at speed {speed}')

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return True

    def handle_move_backward(self, parameters: Dict[str, Any]) -> bool:
        """Handle move backward action"""
        duration = parameters.get('duration', self.default_movement_duration)
        speed = parameters.get('speed', self.default_linear_speed)

        twist_msg = Twist()
        twist_msg.linear.x = -speed

        self.get_logger().info(f'Moving backward for {duration}s at speed {speed}')

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return True

    def handle_turn_left(self, parameters: Dict[str, Any]) -> bool:
        """Handle turn left action"""
        duration = parameters.get('duration', self.default_movement_duration)
        speed = parameters.get('speed', self.default_angular_speed)

        twist_msg = Twist()
        twist_msg.angular.z = speed

        self.get_logger().info(f'Turning left for {duration}s at speed {speed}')

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return True

    def handle_turn_right(self, parameters: Dict[str, Any]) -> bool:
        """Handle turn right action"""
        duration = parameters.get('duration', self.default_movement_duration)
        speed = parameters.get('speed', self.default_angular_speed)

        twist_msg = Twist()
        twist_msg.angular.z = -speed

        self.get_logger().info(f'Turning right for {duration}s at speed {speed}')

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return True

    def handle_stop(self, parameters: Dict[str, Any]) -> bool:
        """Handle stop action"""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('Robot stopped')
        return True

    def handle_pick_up(self, parameters: Dict[str, Any]) -> bool:
        """Handle pick up action"""
        self.get_logger().info('Attempting to pick up object')
        # In simulation, just wait
        time.sleep(2.0)
        return True

    def handle_put_down(self, parameters: Dict[str, Any]) -> bool:
        """Handle put down action"""
        self.get_logger().info('Putting down object')
        # In simulation, just wait
        time.sleep(2.0)
        return True

    def handle_wave(self, parameters: Dict[str, Any]) -> bool:
        """Handle wave action"""
        self.get_logger().info('Waving gesture')
        # Simulate waving motion
        for i in range(5):
            self.simulate_arm_motion('wave', i * 0.5)
            time.sleep(0.3)
        return True

    def handle_dance(self, parameters: Dict[str, Any]) -> bool:
        """Handle dance action"""
        self.get_logger().info('Dancing')
        # Simulate dancing motion
        for i in range(8):
            self.simulate_arm_motion('dance', i * 0.4)
            time.sleep(0.2)
        return True

    def handle_follow_me(self, parameters: Dict[str, Any]) -> bool:
        """Handle follow me action"""
        self.get_logger().info('Activating follow me mode')
        # In simulation, just acknowledge
        return True

    def handle_greet_user(self, parameters: Dict[str, Any]) -> bool:
        """Handle greet user action"""
        self.get_logger().info('Greeting user')
        # In simulation, just acknowledge
        return True

    def handle_take_picture(self, parameters: Dict[str, Any]) -> bool:
        """Handle take picture action"""
        self.get_logger().info('Taking picture')
        # In simulation, just acknowledge
        return True

    def handle_go_to_location(self, parameters: Dict[str, Any]) -> bool:
        """Handle go to location action"""
        location = parameters.get('location', 'unknown')

        # Simple location mapping
        location_coords = {
            'kitchen': (2.0, 1.0),
            'living_room': (0.0, 2.0),
            'bedroom': (-2.0, 1.0),
            'office': (-1.0, -2.0),
            'home': (0.0, 0.0),
            'charger': (0.0, 0.0)
        }

        if location in location_coords:
            target_x, target_y = location_coords[location]
            return self.navigate_to_position(target_x, target_y)
        else:
            self.get_logger().warning(f'Unknown location: {location}')
            return False

    def handle_look_at(self, parameters: Dict[str, Any]) -> bool:
        """Handle look at action"""
        target = parameters.get('object', 'unknown')
        self.get_logger().info(f'Looking at {target}')
        # In simulation, just acknowledge
        return True

    def handle_grab_object(self, parameters: Dict[str, Any]) -> bool:
        """Handle grab object action"""
        self.get_logger().info('Grabbing object')
        # In simulation, just wait
        time.sleep(2.0)
        return True

    def handle_release_object(self, parameters: Dict[str, Any]) -> bool:
        """Handle release object action"""
        self.get_logger().info('Releasing object')
        # In simulation, just wait
        time.sleep(2.0)
        return True

    def handle_wait(self, parameters: Dict[str, Any]) -> bool:
        """Handle wait action"""
        duration = parameters.get('duration', 5.0)
        self.get_logger().info(f'Waiting for {duration} seconds')
        time.sleep(duration)
        return True

    def handle_return_home(self, parameters: Dict[str, Any]) -> bool:
        """Handle return home action"""
        self.get_logger().info('Returning to home position')
        return self.navigate_to_position(0.0, 0.0)

    def handle_general_command(self, parameters: Dict[str, Any]) -> bool:
        """Handle general command"""
        command_text = parameters.get('command_text', '')
        self.get_logger().info(f'Processing general command: {command_text}')
        # For general commands, we might need additional processing
        return True

    def navigate_to_position(self, x: float, y: float) -> bool:
        """
        Navigate to a specific position using navigation stack
        """
        if not self.navigation_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation server not available')
            return False

        from nav2_msgs.action import NavigateToPose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Send navigation goal
        self.get_logger().info(f'Navigating to ({x}, {y})')

        try:
            goal_future = self.navigation_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=self.action_timeout)

            if goal_future.result() is not None:
                return True
            else:
                self.get_logger().error('Navigation goal timed out')
                return False
        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')
            return False

    def simulate_arm_motion(self, motion_type: str, phase: float):
        """
        Simulate arm motion for behaviors
        """
        joint_msg = JointState()
        joint_msg.name = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3']

        if motion_type == 'wave':
            joint_msg.position = [math.sin(phase), math.cos(phase), 0.0]
        elif motion_type == 'dance':
            joint_msg.position = [math.sin(phase * 2), 0.0, math.cos(phase)]

        self.joint_state_pub.publish(joint_msg)

    def log_action_completion(self, action_id: str, success: bool):
        """
        Log action completion and update history
        """
        with self.action_lock:
            if action_id in self.active_actions:
                action_record = {
                    'action_id': action_id,
                    'action_type': self.active_actions[action_id]['request']['action'],
                    'success': success,
                    'start_time': self.active_actions[action_id]['start_time'],
                    'end_time': time.time(),
                    'duration': time.time() - self.active_actions[action_id]['start_time'],
                    'parameters': self.active_actions[action_id]['request']['parameters']
                }

                self.action_history.append(action_record)

                # Keep only recent history
                if len(self.action_history) > 100:
                    self.action_history = self.action_history[-100:]

                self.get_logger().info(f'Action {action_id} completed: {"SUCCESS" if success else "FAILURE"}')

    def get_action_status(self) -> Dict[str, Any]:
        """
        Get current status of action execution system
        """
        with self.action_lock:
            active_count = len([a for a in self.active_actions.values()
                              if a['status'] == ActionResult.IN_PROGRESS])
            queued_count = len(self.action_queue)

            status = {
                'active_actions': active_count,
                'queued_actions': queued_count,
                'total_completed': len(self.action_history),
                'recent_success_rate': self.calculate_recent_success_rate(),
                'active_action_details': {}
            }

            for action_id, action_data in self.active_actions.items():
                if action_data['status'] == ActionResult.IN_PROGRESS:
                    status['active_action_details'][action_id] = {
                        'action': action_data['request']['action'],
                        'elapsed_time': time.time() - action_data['start_time'],
                        'parameters': action_data['request']['parameters']
                    }

        return status

    def calculate_recent_success_rate(self) -> float:
        """
        Calculate success rate of recent actions
        """
        if not self.action_history:
            return 0.0

        recent_actions = self.action_history[-20:]  # Last 20 actions
        successful = sum(1 for action in recent_actions if action['success'])

        return successful / len(recent_actions) if recent_actions else 0.0


def main(args=None):
    """
    Main function to run the action execution system
    """
    rclpy.init(args=args)

    node = ActionExecutionSystem()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Action Execution System shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()