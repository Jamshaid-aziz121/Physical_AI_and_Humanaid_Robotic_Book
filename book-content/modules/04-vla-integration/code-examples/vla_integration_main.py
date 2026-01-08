#!/usr/bin/env python3
"""
VLA Integration System - Main Integration Script

This script brings together all components of the Vision-Language-Action integration system:
- Voice-to-Text Converter
- Intent Recognizer
- Voice-to-Action Pipeline
- Action Execution System
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import sys
import signal
import time


def main(args=None):
    """
    Main function to initialize and run the complete VLA integration system
    """
    rclpy.init(args=args)

    # Import all the components we've created
    try:
        from voice_to_text_converter import VoiceToTextConverter, VoiceCommandProcessor
        from intent_recognizer import IntentRecognizer
        from voice_to_action_pipeline import VoiceToActionPipeline
        from action_execution_system import ActionExecutionSystem
    except ImportError as e:
        print(f"Error importing components: {e}")
        print("Make sure all the required Python files are in the same directory.")
        return 1

    # Create all nodes
    print("Initializing VLA Integration System components...")

    # Create nodes
    voice_converter = VoiceToTextConverter()
    voice_processor = VoiceCommandProcessor()
    intent_recognizer = IntentRecognizer()
    voice_action_pipeline = VoiceToActionPipeline()
    action_execution_system = ActionExecutionSystem()

    # Create multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=5)

    # Add all nodes to executor
    executor.add_node(voice_converter)
    executor.add_node(voice_processor)
    executor.add_node(intent_recognizer)
    executor.add_node(voice_action_pipeline)
    executor.add_node(action_execution_system)

    print("VLA Integration System initialized successfully!")
    print("\nSystem Components Active:")
    print("- Voice-to-Text Converter: Listening for voice commands")
    print("- Voice Command Processor: Processing transcribed text")
    print("- Intent Recognizer: Converting text to robot actions")
    print("- Voice-to-Action Pipeline: Executing robot actions")
    print("- Action Execution System: Managing action execution")
    print("\nReady to accept voice commands!")

    # Handle graceful shutdown
    def signal_handler(sig, frame):
        print('\nShutting down VLA Integration System...')
        executor.shutdown()
        voice_converter.shutdown()
        voice_converter.destroy_node()
        voice_processor.destroy_node()
        intent_recognizer.destroy_node()
        voice_action_pipeline.destroy_node()
        action_execution_system.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        print("\nStarting VLA Integration System - Press Ctrl+C to stop")
        executor.spin()
    except KeyboardInterrupt:
        print('\nShutting down VLA Integration System...')
    finally:
        executor.shutdown()
        voice_converter.shutdown()
        voice_converter.destroy_node()
        voice_processor.destroy_node()
        intent_recognizer.destroy_node()
        voice_action_pipeline.destroy_node()
        action_execution_system.destroy_node()
        rclpy.shutdown()

    print("VLA Integration System shutdown complete.")
    return 0


if __name__ == '__main__':
    sys.exit(main())