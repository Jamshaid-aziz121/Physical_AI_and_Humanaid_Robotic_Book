#!/usr/bin/env python3
"""
Voice-to-Text Converter for Robotics Command Processing

This module implements voice-to-text conversion for processing voice commands
in the Physical AI & Humanoid Robotics Book project.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import speech_recognition as sr
import threading
import queue
import time
import json
from typing import Optional, Dict, Any


class VoiceToTextConverter(Node):
    """
    Node that converts voice commands to text for robotics command processing
    """
    def __init__(self):
        super().__init__('voice_to_text_converter')

        # Initialize speech recognizer and microphone
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Set speech recognition parameters
        self.recognizer.energy_threshold = 300  # Adjust based on ambient noise
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8  # Seconds of silence to wait for
        self.recognizer.phrase_time_limit = 10  # Max seconds for a phrase

        # Publishers
        self.text_pub = self.create_publisher(String, 'voice_text', 10)
        self.command_pub = self.create_publisher(String, 'robot_command', 10)

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Internal state
        self.command_queue = queue.Queue()
        self.listening_active = True
        self.last_transcription = ""

        # Parameters for customization
        self.declare_parameter('language', 'en-US')
        self.declare_parameter('timeout', 5)
        self.declare_parameter('phrase_time_limit', 10)
        self.declare_parameter('energy_threshold', 300)

        # Get parameters
        self.language = self.get_parameter('language').value
        self.timeout = self.get_parameter('timeout').value
        self.phrase_time_limit = self.get_parameter('phrase_time_limit').value
        self.energy_threshold = self.get_parameter('energy_threshold').value

        # Update recognizer settings
        self.recognizer.energy_threshold = self.energy_threshold

        # Start listening thread
        self.listening_thread = threading.Thread(target=self.continuous_listening, daemon=True)
        self.listening_thread.start()

        self.get_logger().info('Voice-to-Text Converter initialized')

    def audio_callback(self, msg):
        """
        Callback for audio data input (if using audio stream instead of microphone)
        """
        # Convert audio data to audio segment
        try:
            audio_data = sr.AudioData(
                msg.data,
                sample_rate=16000,  # Common sample rate
                sample_width=2      # 16-bit audio
            )

            # Recognize speech
            text = self.recognizer.recognize_google(
                audio_data,
                language=self.language
            )

            self.process_transcription(text)
        except sr.UnknownValueError:
            self.get_logger().warning('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Error with speech recognition service: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in audio callback: {e}')

    def continuous_listening(self):
        """
        Continuously listen for voice commands
        """
        self.get_logger().info('Starting continuous voice listening...')

        while self.listening_active:
            try:
                with self.microphone as source:
                    self.get_logger().debug('Listening for voice command...')
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(
                        source,
                        timeout=self.timeout,
                        phrase_time_limit=self.phrase_time_limit
                    )

                # Process the audio
                self.process_audio(audio)

            except sr.WaitTimeoutError:
                # No speech detected within timeout, continue listening
                self.get_logger().debug('No speech detected, continuing to listen...')
                continue
            except sr.UnknownValueError:
                self.get_logger().warning('Could not understand speech')
                continue
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
                time.sleep(1)  # Brief pause before retrying
                continue
            except Exception as e:
                self.get_logger().error(f'Unexpected error in listening: {e}')
                time.sleep(1)  # Brief pause before retrying
                continue

    def process_audio(self, audio):
        """
        Process audio data and convert to text
        """
        try:
            # Recognize speech using Google's speech recognition
            text = self.recognizer.recognize_google(audio, language=self.language)
            self.process_transcription(text)
        except sr.RequestError as e:
            self.get_logger().error(f'Error with speech recognition service: {e}')
        except sr.UnknownValueError:
            self.get_logger().warning('Could not understand audio')

    def process_transcription(self, text: str):
        """
        Process the transcribed text and publish it
        """
        if not text.strip():
            return

        self.get_logger().info(f'Transcribed: "{text}"')

        # Store the transcription
        self.last_transcription = text

        # Create and publish text message
        text_msg = String()
        text_msg.data = text
        self.text_pub.publish(text_msg)

        # Optionally, publish as a robot command (with some processing)
        command_msg = String()
        command_msg.data = self.process_for_robot_command(text)
        self.command_pub.publish(command_msg)

        # Log the processed command
        self.get_logger().info(f'Processed command: {command_msg.data}')

    def process_for_robot_command(self, text: str) -> str:
        """
        Process text to extract robot command format
        """
        # Simple preprocessing of the text
        processed_text = text.lower().strip()

        # Basic command mapping (in a real system, this would be more sophisticated)
        command_mappings = {
            'move forward': 'MOVE_FORWARD',
            'go forward': 'MOVE_FORWARD',
            'move backward': 'MOVE_BACKWARD',
            'go back': 'MOVE_BACKWARD',
            'turn left': 'TURN_LEFT',
            'turn right': 'TURN_RIGHT',
            'stop': 'STOP',
            'halt': 'STOP',
            'pick up': 'PICK_UP',
            'grasp': 'PICK_UP',
            'put down': 'PUT_DOWN',
            'release': 'PUT_DOWN',
            'wave': 'WAVE',
            'dance': 'DANCE',
            'follow me': 'FOLLOW_ME',
            'greet': 'GREET_USER',
            'hello': 'GREET_USER',
            'take picture': 'TAKE_PICTURE',
            'capture': 'TAKE_PICTURE',
        }

        # Check for command mappings
        for phrase, command in command_mappings.items():
            if phrase in processed_text:
                return command

        # If no specific command found, return as GENERAL_COMMAND
        return f'GENERAL_COMMAND: {processed_text}'

    def get_last_transcription(self) -> str:
        """
        Get the last transcribed text
        """
        return self.last_transcription

    def shutdown(self):
        """
        Clean shutdown of the voice converter
        """
        self.listening_active = False
        if self.listening_thread.is_alive():
            self.listening_thread.join(timeout=2.0)


class VoiceCommandProcessor(Node):
    """
    Additional node to process voice commands and convert them to structured robot commands
    """
    def __init__(self):
        super().__init__('voice_command_processor')

        # Subscribe to transcribed text
        self.voice_sub = self.create_subscription(
            String,
            'voice_text',
            self.voice_text_callback,
            10
        )

        # Subscribe to raw commands
        self.command_sub = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10
        )

        # Publish structured robot commands
        self.structured_command_pub = self.create_publisher(
            String,
            'structured_robot_command',
            10
        )

        self.get_logger().info('Voice Command Processor initialized')

    def voice_text_callback(self, msg):
        """
        Process raw transcribed text
        """
        text = msg.data
        self.get_logger().debug(f'Received voice text: {text}')

        # Perform additional processing if needed
        processed_command = self.parse_voice_command(text)

        # Publish structured command
        command_msg = String()
        command_msg.data = json.dumps(processed_command)
        self.structured_command_pub.publish(command_msg)

        self.get_logger().info(f'Published structured command: {processed_command}')

    def command_callback(self, msg):
        """
        Process simple robot command
        """
        command = msg.data
        self.get_logger().debug(f'Received simple command: {command}')

        # Could implement additional logic here
        pass

    def parse_voice_command(self, text: str) -> Dict[str, Any]:
        """
        Parse voice command text into structured format
        """
        # Simple NLP parsing to extract intent and parameters
        text_lower = text.lower()

        # Define command patterns
        patterns = {
            'move_forward': ['forward', 'ahead', 'straight'],
            'move_backward': ['backward', 'back', 'reverse'],
            'turn_left': ['left', 'port'],
            'turn_right': ['right', 'starboard'],
            'stop': ['stop', 'halt', 'pause'],
            'pick_up': ['pick up', 'grasp', 'grab', 'lift'],
            'put_down': ['put down', 'place', 'release', 'drop'],
            'wave': ['wave', 'waving', 'hello', 'hi'],
            'dance': ['dance', 'dancing', 'move'],
            'follow_me': ['follow', 'follow me', 'come'],
            'greet_user': ['hello', 'hi', 'greet', 'hey'],
            'take_picture': ['photo', 'picture', 'capture', 'snapshot'],
        }

        # Identify the primary command
        identified_action = 'unknown'
        confidence = 0.0

        for action, keywords in patterns.items():
            for keyword in keywords:
                if keyword in text_lower:
                    identified_action = action
                    confidence = 0.8  # Set a base confidence
                    break
            if confidence > 0:  # Found a match
                break

        # Extract potential parameters (numbers, locations, objects)
        import re
        numbers = re.findall(r'\d+', text_lower)
        parameters = {
            'distance': float(numbers[0]) if numbers else 1.0,  # Default distance
            'speed': 0.5,  # Default speed
            'confidence': confidence,
            'original_text': text
        }

        # Structure the command
        structured_command = {
            'action': identified_action,
            'parameters': parameters,
            'raw_text': text,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        return structured_command


def main(args=None):
    """
    Main function to run the voice-to-text converter
    """
    rclpy.init(args=args)

    # Create both nodes
    converter = VoiceToTextConverter()
    processor = VoiceCommandProcessor()

    try:
        # Run both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(converter)
        executor.add_node(processor)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    finally:
        converter.shutdown()
        converter.destroy_node()
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()