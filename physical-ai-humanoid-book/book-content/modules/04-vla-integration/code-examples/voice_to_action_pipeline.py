#!/usr/bin/env python3
"""
Voice-to-Action Pipeline for Robotics Command Execution

This module implements the pipeline that converts voice commands to ROS actions,
connecting the intent recognition system to actual robot command execution.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import json
from typing import Dict, Any, Optional
from enum import Enum
import math
import time

from intent_recognizer import RobotAction, IntentResult


class ActionExecutionStatus(Enum):
    """
    Status of action execution
    """
    SUCCESS = "success"
    FAILED = "failed"
    IN_PROGRESS = "in_progress"
    PENDING = "pending"


class VoiceToActionPipeline(Node):
    """
    Node that converts voice commands to ROS actions and executes them
    """
    def __init__(self):
        super().__init__('voice_to_action_pipeline')

        # Subscribe to recognized intents
        self.intent_sub = self.create_subscription(
            String,
            'recognized_intent',
            self.intent_callback,
            10
        )

        # Publishers for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_commands', 10)

        # Action clients for complex movements
        self.navigation_client = ActionClient(self, 'nav2_msgs/action/NavigateToPose', 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, 'control_msgs/action/FollowJointTrajectory', 'manipulator_controller/follow_joint_trajectory')

        # Robot state tracking
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.current_pose = None

        # Action execution tracking
        self.active_actions = {}
        self.action_timeout = 30.0  # seconds

        # Parameters for customization
        self.declare_parameter('default_linear_speed', 0.5)
        self.declare_parameter('default_angular_speed', 0.5)
        self.declare_parameter('default_movement_duration', 2.0)
        self.declare_parameter('action_timeout', 30.0)

        self.default_linear_speed = self.get_parameter('default_linear_speed').value
        self.default_angular_speed = self.get_parameter('default_angular_speed').value
        self.default_movement_duration = self.get_parameter('default_movement_duration').value
        self.action_timeout = self.get_parameter('action_timeout').value

        self.get_logger().info('Voice-to-Action Pipeline initialized')

    def intent_callback(self, msg: String):
        """
        Callback to process recognized intents and execute actions
        """
        try:
            intent_dict = json.loads(msg.data)

            # Create IntentResult object from the received data
            intent_result = IntentResult(
                action=RobotAction(intent_dict['action']),
                confidence=intent_dict['confidence'],
                parameters=intent_dict['parameters'],
                original_command=intent_dict['original_command'],
                extracted_entities=intent_dict['extracted_entities']
            )

            self.get_logger().info(f'Received intent: {intent_result.action.value} with confidence {intent_result.confidence}')

            # Execute the action if confidence is high enough
            if intent_result.confidence >= 0.7:  # High confidence threshold
                self.execute_action(intent_result)
            else:
                self.get_logger().warning(f'Low confidence intent ignored: {intent_result.confidence}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode intent message: {e}')
        except ValueError as e:
            self.get_logger().error(f'Invalid robot action in intent: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing intent: {e}')

    def odom_callback(self, msg: Odometry):
        """
        Callback to track robot pose
        """
        self.current_pose = msg.pose.pose

    def execute_action(self, intent_result: IntentResult) -> bool:
        """
        Execute the action based on the recognized intent
        """
        action = intent_result.action
        params = intent_result.parameters

        self.get_logger().info(f'Executing action: {action.value}')

        # Create unique action ID
        action_id = f"{action.value}_{int(time.time() * 1000)}"

        # Track action execution
        self.active_actions[action_id] = {
            'status': ActionExecutionStatus.IN_PROGRESS,
            'start_time': time.time(),
            'intent_result': intent_result
        }

        try:
            if action in [RobotAction.MOVE_FORWARD, RobotAction.MOVE_BACKWARD]:
                success = self.execute_movement_action(action, params)
            elif action in [RobotAction.TURN_LEFT, RobotAction.TURN_RIGHT]:
                success = self.execute_rotation_action(action, params)
            elif action == RobotAction.STOP:
                success = self.execute_stop_action()
            elif action in [RobotAction.PICK_UP, RobotAction.PUT_DOWN, RobotAction.GRAB_OBJECT, RobotAction.RELEASE_OBJECT]:
                success = self.execute_manipulation_action(action, params)
            elif action in [RobotAction.GO_TO_LOCATION, RobotAction.MOVE_TO, RobotAction.RETURN_HOME]:
                success = self.execute_navigation_action(action, params)
            elif action in [RobotAction.WAVE, RobotAction.DANCE]:
                success = self.execute_behavior_action(action, params)
            elif action == RobotAction.LOOK_AT:
                success = self.execute_head_movement_action(action, params)
            elif action in [RobotAction.TAKE_PICTURE, RobotAction.FOLLOW_ME]:
                success = self.execute_special_action(action, params)
            elif action == RobotAction.WAIT:
                success = self.execute_wait_action(params)
            else:
                self.get_logger().warning(f'Unknown action: {action.value}')
                success = False

            # Update action status
            if action_id in self.active_actions:
                self.active_actions[action_id]['status'] = ActionExecutionStatus.SUCCESS if success else ActionExecutionStatus.FAILED
                self.active_actions[action_id]['completed'] = True

            return success

        except Exception as e:
            self.get_logger().error(f'Error executing action {action.value}: {e}')
            if action_id in self.active_actions:
                self.active_actions[action_id]['status'] = ActionExecutionStatus.FAILED
                self.active_actions[action_id]['error'] = str(e)
            return False

    def execute_movement_action(self, action: RobotAction, params: Dict[str, Any]) -> bool:
        """
        Execute movement actions (forward/backward)
        """
        twist_msg = Twist()

        if action == RobotAction.MOVE_FORWARD:
            twist_msg.linear.x = self.default_linear_speed
        elif action == RobotAction.MOVE_BACKWARD:
            twist_msg.linear.x = -self.default_linear_speed
        else:
            return False

        # Apply duration if specified
        duration = params.get('duration', self.default_movement_duration)

        self.get_logger().info(f'Moving {action.value} for {duration} seconds')

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot after movement
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return True

    def execute_rotation_action(self, action: RobotAction, params: Dict[str, Any]) -> bool:
        """
        Execute rotation actions (left/right)
        """
        twist_msg = Twist()

        if action == RobotAction.TURN_LEFT:
            twist_msg.angular.z = self.default_angular_speed
        elif action == RobotAction.TURN_RIGHT:
            twist_msg.angular.z = -self.default_angular_speed
        else:
            return False

        # Apply duration if specified
        duration = params.get('duration', self.default_movement_duration)

        self.get_logger().info(f'Turning {action.value} for {duration} seconds')

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot after rotation
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return True

    def execute_stop_action(self) -> bool:
        """
        Execute stop action
        """
        twist_msg = Twist()  # Zero velocities
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Robot stopped')
        return True

    def execute_manipulation_action(self, action: RobotAction, params: Dict[str, Any]) -> bool:
        """
        Execute manipulation actions (pick up, put down, grab, release)
        """
        # For simulation, we'll just log the action
        self.get_logger().info(f'Executing manipulation action: {action.value}')

        # In a real system, this would involve:
        # - Planning manipulator trajectories
        # - Controlling grippers/joints
        # - Using sensor feedback

        if action in [RobotAction.PICK_UP, RobotAction.GRAB_OBJECT]:
            self.get_logger().info('Attempting to pick up object')
        elif action in [RobotAction.PUT_DOWN, RobotAction.RELEASE_OBJECT]:
            self.get_logger().info('Releasing object')

        # Simulate action completion
        time.sleep(2.0)  # Simulate action time

        return True

    def execute_navigation_action(self, action: RobotAction, params: Dict[str, Any]) -> bool:
        """
        Execute navigation actions (go to location, return home)
        """
        # Check if navigation server is available
        if not self.navigation_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation server not available')
            return False

        # Determine target location
        target_x, target_y = 0.0, 0.0  # Default to origin/home

        if action == RobotAction.GO_TO_LOCATION or action == RobotAction.MOVE_TO:
            # Extract location from parameters
            location = params.get('location', 'unknown')

            # Simple location mapping (in a real system, this would use a map)
            location_map = {
                'kitchen': (2.0, 1.0),
                'living_room': (0.0, 2.0),
                'bedroom': (-2.0, 1.0),
                'office': (-1.0, -2.0),
                'home': (0.0, 0.0),
                'charger': (0.0, 0.0)  # Same as home for now
            }

            if location in location_map:
                target_x, target_y = location_map[location]
            else:
                # If location not found, try to extract coordinates from entities
                if 'coordinates' in params:
                    coords = params['coordinates']
                    if isinstance(coords, dict) and 'x' in coords and 'y' in coords:
                        target_x, target_y = coords['x'], coords['y']
                    elif isinstance(coords, list) and len(coords) >= 2:
                        target_x, target_y = float(coords[0]), float(coords[1])

        elif action == RobotAction.RETURN_HOME:
            target_x, target_y = 0.0, 0.0  # Return to origin

        # Create navigation goal
        from nav2_msgs.action import NavigateToPose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0
        # Set orientation to face forward (z=0, w=1)
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Send navigation goal
        self.get_logger().info(f'Navigating to ({target_x}, {target_y}) for {action.value}')

        goal_future = self.navigation_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        # Wait for result with timeout
        goal_future.add_done_callback(lambda future: self.navigation_result_callback(future))

        return True

    def execute_behavior_action(self, action: RobotAction, params: Dict[str, Any]) -> bool:
        """
        Execute behavior actions (wave, dance)
        """
        self.get_logger().info(f'Executing behavior: {action.value}')

        # For simulation, we'll just log the action
        # In a real system, this would involve:
        # - Executing predefined joint trajectories
        # - Coordinating multiple actuators

        if action == RobotAction.WAVE:
            self.get_logger().info('Waving gesture')
            # Simulate wave motion
            for i in range(3):
                self.simulate_arm_motion('wave', i * 0.5)
                time.sleep(0.5)

        elif action == RobotAction.DANCE:
            self.get_logger().info('Dancing')
            # Simulate dance motion
            for i in range(4):
                self.simulate_arm_motion('dance', i * 0.3)
                time.sleep(0.3)

        return True

    def execute_head_movement_action(self, action: RobotAction, params: Dict[str, Any]) -> bool:
        """
        Execute head movement actions (look at)
        """
        self.get_logger().info(f'Looking at: {params.get("object", "unknown")}')

        # In a real system, this would:
        # - Move head/pan-tilt mechanism to look at object
        # - Use vision to center object in view
        # - Potentially track moving objects

        # For simulation, just log the action
        return True

    def execute_special_action(self, action: RobotAction, params: Dict[str, Any]) -> bool:
        """
        Execute special actions (take picture, follow me)
        """
        if action == RobotAction.TAKE_PICTURE:
            self.get_logger().info('Taking picture')
            # In a real system, this would trigger camera capture
            return True

        elif action == RobotAction.FOLLOW_ME:
            self.get_logger().info('Following person')
            # In a real system, this would activate person-following algorithm
            return True

        return False

    def execute_wait_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute wait action
        """
        duration = params.get('duration', 5.0)  # Default 5 seconds
        self.get_logger().info(f'Waiting for {duration} seconds')
        time.sleep(duration)
        return True

    def simulate_arm_motion(self, motion_type: str, phase: float):
        """
        Simulate arm motion for behaviors
        """
        # This is a placeholder for simulating arm movements
        # In a real system, this would publish joint commands
        joint_msg = JointState()
        joint_msg.name = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3']
        if motion_type == 'wave':
            joint_msg.position = [math.sin(phase), math.cos(phase), 0.0]
        elif motion_type == 'dance':
            joint_msg.position = [math.sin(phase * 2), 0.0, math.cos(phase)]

        self.joint_state_pub.publish(joint_msg)

    def navigation_feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback
        """
        self.get_logger().debug('Navigation in progress...')

    def navigation_result_callback(self, future):
        """
        Callback for navigation result
        """
        try:
            goal_handle = future.result()
            result = goal_handle.get_result_async()
            self.get_logger().info('Navigation completed')
        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')

    def cleanup_expired_actions(self):
        """
        Clean up expired actions from the tracking dictionary
        """
        current_time = time.time()
        expired_ids = []

        for action_id, action_data in self.active_actions.items():
            if (current_time - action_data['start_time']) > self.action_timeout:
                expired_ids.append(action_id)

        for action_id in expired_ids:
            del self.active_actions[action_id]
            self.get_logger().warning(f'Cleaned up expired action: {action_id}')

    def get_active_actions_status(self) -> Dict[str, Any]:
        """
        Get status of all active actions
        """
        self.cleanup_expired_actions()

        status = {
            'total_actions': len(self.active_actions),
            'in_progress': 0,
            'completed_success': 0,
            'completed_failed': 0,
            'actions': {}
        }

        for action_id, action_data in self.active_actions.items():
            status['actions'][action_id] = {
                'action': action_data['intent_result'].action.value,
                'confidence': action_data['intent_result'].confidence,
                'status': action_data['status'].value,
                'elapsed_time': time.time() - action_data['start_time']
            }

            if action_data['status'] == ActionExecutionStatus.IN_PROGRESS:
                status['in_progress'] += 1
            elif action_data['status'] == ActionExecutionStatus.SUCCESS:
                status['completed_success'] += 1
            elif action_data['status'] == ActionExecutionStatus.FAILED:
                status['completed_failed'] += 1

        return status


def main(args=None):
    """
    Main function to run the voice-to-action pipeline
    """
    rclpy.init(args=args)

    node = VoiceToActionPipeline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Voice-to-Action Pipeline shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()