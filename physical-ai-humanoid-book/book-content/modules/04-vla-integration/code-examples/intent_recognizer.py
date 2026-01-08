#!/usr/bin/env python3
"""
Intent Recognizer for Voice Commands in Robotics

This module implements an intent recognition system that interprets voice commands
for robotics applications. It maps natural language to specific robot actions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import re
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
from enum import Enum
import math


class RobotAction(Enum):
    """
    Enum for all possible robot actions
    """
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    STOP = "stop"
    PICK_UP = "pick_up"
    PUT_DOWN = "put_down"
    WAVE = "wave"
    DANCE = "dance"
    FOLLOW_ME = "follow_me"
    GO_TO_LOCATION = "go_to_location"
    FETCH_OBJECT = "fetch_object"
    TAKE_PICTURE = "take_picture"
    GREET_USER = "greet_user"
    MOVE_TO = "move_to"
    LOOK_AT = "look_at"
    GRAB_OBJECT = "grab_object"
    RELEASE_OBJECT = "release_object"
    WAIT = "wait"
    EXPLORE_AREA = "explore_area"
    CHARGE_BATTERY = "charge_battery"
    RETURN_HOME = "return_home"
    UNKNOWN = "unknown"


@dataclass
class IntentResult:
    """
    Result of intent recognition
    """
    action: RobotAction
    confidence: float  # 0.0 to 1.0
    parameters: Dict[str, Any]
    original_command: str
    extracted_entities: List[str]


class IntentRecognizer(Node):
    """
    Node that recognizes intents from voice commands
    """
    def __init__(self):
        super().__init__('intent_recognizer')

        # Subscribe to voice text
        self.voice_text_sub = self.create_subscription(
            String,
            'voice_text',
            self.voice_text_callback,
            10
        )

        # Publish recognized intents
        self.intent_pub = self.create_publisher(String, 'recognized_intent', 10)
        self.confidence_pub = self.create_publisher(Bool, 'high_confidence_intent', 10)

        # Publishers for specific robot actions
        self.move_pub = self.create_publisher(String, 'move_command', 10)
        self.action_pub = self.create_publisher(String, 'action_command', 10)

        # Intent recognition patterns
        self.intent_patterns = self.define_intent_patterns()

        # Parameters for customization
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('enable_contextual_understanding', True)
        self.declare_parameter('entity_extraction_enabled', True)

        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.contextual_enabled = self.get_parameter('enable_contextual_understanding').value
        self.entity_extraction_enabled = self.get_parameter('entity_extraction_enabled').value

        self.get_logger().info('Intent Recognizer initialized')

    def define_intent_patterns(self) -> Dict[RobotAction, List[Dict[str, Any]]]:
        """
        Define patterns for recognizing different intents
        """
        patterns = {}

        # Movement patterns
        patterns[RobotAction.MOVE_FORWARD] = [
            {'pattern': r'go\s+(?:forward|ahead|straight)', 'weight': 0.9},
            {'pattern': r'move\s+(?:forward|ahead)', 'weight': 0.9},
            {'pattern': r'go\s+(?:straight|on)', 'weight': 0.8},
            {'pattern': r'forward', 'weight': 0.7},
            {'pattern': r'up', 'weight': 0.6, 'contexts': ['direction']},
        ]

        patterns[RobotAction.MOVE_BACKWARD] = [
            {'pattern': r'go\s+back(?:ward)?', 'weight': 0.9},
            {'pattern': r'move\s+back(?:ward)?', 'weight': 0.9},
            {'pattern': r'reverse', 'weight': 0.8},
            {'pattern': r'back', 'weight': 0.7},
        ]

        patterns[RobotAction.TURN_LEFT] = [
            {'pattern': r'turn\s+left', 'weight': 0.9},
            {'pattern': r'rotate\s+left', 'weight': 0.8},
            {'pattern': r'pivot\s+left', 'weight': 0.8},
            {'pattern': r'left', 'weight': 0.6, 'contexts': ['direction']},
        ]

        patterns[RobotAction.TURN_RIGHT] = [
            {'pattern': r'turn\s+right', 'weight': 0.9},
            {'pattern': r'rotate\s+right', 'weight': 0.8},
            {'pattern': r'pivot\s+right', 'weight': 0.8},
            {'pattern': r'right', 'weight': 0.6, 'contexts': ['direction']},
        ]

        patterns[RobotAction.STOP] = [
            {'pattern': r'stop', 'weight': 0.9},
            {'pattern': r'halt', 'weight': 0.8},
            {'pattern': r'pause', 'weight': 0.8},
            {'pattern': r'freeze', 'weight': 0.7},
        ]

        patterns[RobotAction.PICK_UP] = [
            {'pattern': r'pick\s+up\s+(\w+)', 'weight': 0.9, 'extract': 'object'},
            {'pattern': r'grab\s+(\w+)', 'weight': 0.9, 'extract': 'object'},
            {'pattern': r'lift\s+(\w+)', 'weight': 0.8, 'extract': 'object'},
            {'pattern': r'get\s+the\s+(\w+)', 'weight': 0.8, 'extract': 'object'},
            {'pattern': r'grasp\s+(\w+)', 'weight': 0.8, 'extract': 'object'},
        ]

        patterns[RobotAction.PUT_DOWN] = [
            {'pattern': r'put\s+down', 'weight': 0.9},
            {'pattern': r'drop', 'weight': 0.8},
            {'pattern': r'release', 'weight': 0.8},
            {'pattern': r'place', 'weight': 0.7},
        ]

        patterns[RobotAction.WAVE] = [
            {'pattern': r'wave', 'weight': 0.9},
            {'pattern': r'(?:please\s+)?wave\s+to', 'weight': 0.9},
            {'pattern': r'waving', 'weight': 0.7},
        ]

        patterns[RobotAction.DANCE] = [
            {'pattern': r'dance', 'weight': 0.9},
            {'pattern': r'(?:please\s+)?dance', 'weight': 0.9},
            {'pattern': r'dancing', 'weight': 0.7},
        ]

        patterns[RobotAction.FOLLOW_ME] = [
            {'pattern': r'follow\s+me', 'weight': 0.9},
            {'pattern': r'come\s+with\s+me', 'weight': 0.8},
            {'pattern': r'follow', 'weight': 0.7, 'contexts': ['person']},
        ]

        patterns[RobotAction.GO_TO_LOCATION] = [
            {'pattern': r'go\s+to\s+(?:the\s+)?(\w+)', 'weight': 0.9, 'extract': 'location'},
            {'pattern': r'move\s+to\s+(?:the\s+)?(\w+)', 'weight': 0.9, 'extract': 'location'},
            {'pattern': r'go\s+over\s+to\s+(?:the\s+)?(\w+)', 'weight': 0.8, 'extract': 'location'},
            {'pattern': r'take\s+me\s+to\s+(?:the\s+)?(\w+)', 'weight': 0.8, 'extract': 'location'},
        ]

        patterns[RobotAction.FETCH_OBJECT] = [
            {'pattern': r'fetch\s+(?:the\s+)?(\w+)', 'weight': 0.9, 'extract': 'object'},
            {'pattern': r'bring\s+me\s+(?:the\s+)?(\w+)', 'weight': 0.9, 'extract': 'object'},
            {'pattern': r'get\s+(?:me\s+)?(?:the\s+)?(\w+)', 'weight': 0.9, 'extract': 'object'},
            {'pattern': r'go\s+get\s+(?:the\s+)?(\w+)', 'weight': 0.8, 'extract': 'object'},
        ]

        patterns[RobotAction.TAKE_PICTURE] = [
            {'pattern': r'take\s+(?:a\s+)?picture', 'weight': 0.9},
            {'pattern': r'capture\s+(?:a\s+)?photo', 'weight': 0.9},
            {'pattern': r'snap(?:shot)?', 'weight': 0.8},
            {'pattern': r'photograph', 'weight': 0.8},
        ]

        patterns[RobotAction.GREET_USER] = [
            {'pattern': r'hello', 'weight': 0.8},
            {'pattern': r'hi', 'weight': 0.8},
            {'pattern': r'greet', 'weight': 0.7},
            {'pattern': r'hey', 'weight': 0.6},
        ]

        patterns[RobotAction.LOOK_AT] = [
            {'pattern': r'look\s+at\s+(?:the\s+)?(\w+)', 'weight': 0.8, 'extract': 'object'},
            {'pattern': r'focus\s+on\s+(?:the\s+)?(\w+)', 'weight': 0.8, 'extract': 'object'},
            {'pattern': r'watch\s+(?:the\s+)?(\w+)', 'weight': 0.7, 'extract': 'object'},
        ]

        patterns[RobotAction.WAIT] = [
            {'pattern': r'wait', 'weight': 0.8},
            {'pattern': r'stand\s+by', 'weight': 0.7},
            {'pattern': r'hold\s+on', 'weight': 0.7},
        ]

        patterns[RobotAction.RETURN_HOME] = [
            {'pattern': r'return\s+home', 'weight': 0.9},
            {'pattern': r'go\s+home', 'weight': 0.9},
            {'pattern': r'go\s+back\s+home', 'weight': 0.8},
            {'pattern': r'return\s+to\s+dock', 'weight': 0.8},
        ]

        return patterns

    def voice_text_callback(self, msg: String):
        """
        Callback to process voice text and recognize intent
        """
        command_text = msg.data
        self.get_logger().debug(f'Received voice command: {command_text}')

        # Recognize intent from the command
        intent_result = self.recognize_intent(command_text)

        # Publish the recognized intent
        self.publish_intent(intent_result)

        # Process based on action type
        self.process_intent_based_action(intent_result)

    def recognize_intent(self, text: str) -> IntentResult:
        """
        Recognize intent from text using pattern matching and confidence scoring
        """
        text_lower = text.lower().strip()

        best_match = None
        best_confidence = 0.0
        best_action = RobotAction.UNKNOWN
        best_parameters = {}
        extracted_entities = []

        # Try to match each action pattern
        for action, patterns in self.intent_patterns.items():
            for pattern_info in patterns:
                pattern = pattern_info['pattern']
                weight = pattern_info['weight']

                match = re.search(pattern, text_lower, re.IGNORECASE)
                if match:
                    # Calculate confidence based on pattern weight and match quality
                    confidence = weight

                    # Extract parameters if specified
                    parameters = {}
                    if 'extract' in pattern_info and match.groups():
                        entity_value = match.group(1)
                        entity_type = pattern_info['extract']
                        parameters[entity_type] = entity_value
                        extracted_entities.append(entity_value)

                    # Boost confidence if this seems like a clear match
                    if len(match.group(0)) > 0.7 * len(text_lower):
                        confidence *= 1.1  # Boost for longer matches

                    # Ensure confidence doesn't exceed 1.0
                    confidence = min(confidence, 1.0)

                    # Update best match if this one is better
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_action = action
                        best_parameters = parameters
                        # Create a copy of entities list to avoid mutation issues
                        extracted_entities = extracted_entities.copy()
                        # Add any new entities from this match
                        for i in range(1, len(match.groups()) + 1):
                            try:
                                entity = match.group(i)
                                if entity and entity not in extracted_entities:
                                    extracted_entities.append(entity)
                            except IndexError:
                                break

        # If no pattern matched well, assign a low confidence to unknown action
        if best_confidence < 0.3:
            best_action = RobotAction.UNKNOWN
            best_confidence = 0.1

        return IntentResult(
            action=best_action,
            confidence=best_confidence,
            parameters=best_parameters,
            original_command=text,
            extracted_entities=extracted_entities
        )

    def publish_intent(self, intent_result: IntentResult):
        """
        Publish the recognized intent
        """
        # Create intent message
        intent_msg = String()
        intent_dict = {
            'action': intent_result.action.value,
            'confidence': intent_result.confidence,
            'parameters': intent_result.parameters,
            'original_command': intent_result.original_command,
            'extracted_entities': intent_result.extracted_entities
        }
        intent_msg.data = json.dumps(intent_dict)

        self.intent_pub.publish(intent_msg)
        self.get_logger().info(f'Published intent: {intent_dict}')

        # Publish high confidence flag
        high_conf_flag = Bool()
        high_conf_flag.data = intent_result.confidence >= self.confidence_threshold
        self.confidence_pub.publish(high_conf_flag)

    def process_intent_based_action(self, intent_result: IntentResult):
        """
        Process the intent and publish appropriate robot commands
        """
        action = intent_result.action
        params = intent_result.parameters

        # Map actions to appropriate command publications
        if action in [RobotAction.MOVE_FORWARD, RobotAction.MOVE_BACKWARD,
                      RobotAction.TURN_LEFT, RobotAction.TURN_RIGHT, RobotAction.STOP]:
            # Movement commands
            move_cmd = {
                'action': action.value,
                'parameters': params,
                'confidence': intent_result.confidence
            }
            move_msg = String()
            move_msg.data = json.dumps(move_cmd)
            self.move_pub.publish(move_msg)

        elif action in [RobotAction.PICK_UP, RobotAction.PUT_DOWN, RobotAction.WAVE,
                        RobotAction.DANCE, RobotAction.GREET_USER, RobotAction.TAKE_PICTURE]:
            # Action commands
            action_cmd = {
                'action': action.value,
                'parameters': params,
                'confidence': intent_result.confidence
            }
            action_msg = String()
            action_msg.data = json.dumps(action_cmd)
            self.action_pub.publish(action_msg)

        else:
            # For other commands, publish to general action topic
            gen_cmd = {
                'action': action.value,
                'parameters': params,
                'confidence': intent_result.confidence
            }
            gen_msg = String()
            gen_msg.data = json.dumps(gen_cmd)
            self.action_pub.publish(gen_msg)

    def get_supported_intents(self) -> List[str]:
        """
        Get list of supported intent types
        """
        return [action.value for action in RobotAction]


class IntentTrainingHelper:
    """
    Helper class to train and improve intent recognition
    """
    def __init__(self):
        self.training_data = []
        self.pattern_weights = {}

    def add_training_example(self, text: str, expected_action: RobotAction):
        """
        Add a training example to improve recognition
        """
        self.training_data.append({
            'text': text,
            'expected_action': expected_action
        })

    def evaluate_pattern_effectiveness(self) -> Dict[str, float]:
        """
        Evaluate how well current patterns perform on training data
        """
        results = {}
        for action, patterns in self.intent_patterns.items():
            matches = 0
            total = 0

            for example in self.training_data:
                if example['expected_action'] == action:
                    total += 1
                    for pattern_info in patterns:
                        pattern = pattern_info['pattern']
                        if re.search(pattern, example['text'], re.IGNORECASE):
                            matches += 1
                            break

            if total > 0:
                results[action.value] = matches / total
            else:
                results[action.value] = 0.0

        return results


def main(args=None):
    """
    Main function to run the intent recognizer
    """
    rclpy.init(args=args)

    node = IntentRecognizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Intent Recognizer shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()