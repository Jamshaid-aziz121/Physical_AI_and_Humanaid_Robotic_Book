#!/bin/bash

# Setup script for LLM Integration Framework for Robotics Applications
# This script sets up the environment for integrating Large Language Models with robotics systems

set -e  # Exit on any error

echo "========================================="
echo "Setting up LLM Integration Framework for Robotics"
echo "========================================="

# Check if Python 3.10+ is available
PYTHON_VERSION=$(python3 --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
if [ -z "$PYTHON_VERSION" ]; then
    echo "Error: Python 3 not found"
    exit 1
fi

MAJOR_VERSION=$(echo $PYTHON_VERSION | cut -d. -f1)
MINOR_VERSION=$(echo $PYTHON_VERSION | cut -d. -f2)

if [ "$MAJOR_VERSION" -lt 3 ] || ([ "$MAJOR_VERSION" -eq 3 ] && [ "$MINOR_VERSION" -lt 10 ]); then
    echo "Error: Python 3.10 or higher is required. Found: $PYTHON_VERSION"
    exit 1
fi

echo "Python version check passed: $PYTHON_VERSION"

# Create project directory structure
LLM_ROOT="$HOME/llm_robotics"
mkdir -p "$LLM_ROOT"
cd "$LLM_ROOT"

echo "Created LLM Robotics directory: $LLM_ROOT"

# Create virtual environment
echo "Creating Python virtual environment..."
python3 -m venv llm_env
source llm_env/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install core dependencies for LLM integration
echo "Installing core dependencies..."

# Install AI/ML libraries
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install transformers
pip install openai
pip install anthropic
pip install cohere-toolkit
pip install sentence-transformers
pip install numpy pandas scikit-learn

# Install ROS 2 Python libraries
pip install rclpy

# Install audio processing libraries
pip install pyaudio speechrecognition sounddevice librosa

# Install other essential libraries
pip install python-dotenv requests aiohttp asyncio

echo "Core dependencies installed"

# Create configuration directory
mkdir -p config
touch config/.env  # For API keys

# Create the LLM integration framework
cat << 'EOF' > llm_robotics_framework.py
#!/usr/bin/env python3
"""
LLM Integration Framework for Robotics Applications

This module provides the core framework for integrating Large Language Models
with robotics systems for voice-driven control and task planning.
"""

import os
import json
import asyncio
import logging
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass
from abc import ABC, abstractmethod

# Import AI libraries
try:
    import openai
    from openai import AsyncOpenAI
except ImportError:
    print("OpenAI library not available. Install with: pip install openai")

try:
    import anthropic
    from anthropic import AsyncAnthropic
except ImportError:
    print("Anthropic library not available. Install with: pip install anthropic")

try:
    import cohere
except ImportError:
    print("Cohere library not available. Install with: pip install cohere-toolkit")

# Import audio processing
try:
    import speech_recognition as sr
except ImportError:
    print("SpeechRecognition library not available. Install with: pip install speechrecognition")

# Import ROS 2
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
except ImportError:
    print("ROS 2 libraries not available. Make sure ROS 2 is sourced.")


@dataclass
class RobotCommand:
    """Represents a command for the robot."""
    action: str
    parameters: Dict[str, Any]
    confidence: float
    intent: str


class LLMProvider(ABC):
    """Abstract base class for LLM providers."""

    @abstractmethod
    async def generate_response(self, prompt: str, context: Optional[Dict] = None) -> str:
        """Generate a response from the LLM."""
        pass

    @abstractmethod
    async def process_intent(self, text: str, available_actions: List[str]) -> RobotCommand:
        """Process text and extract robot command."""
        pass


class OpenAILLM(LLMProvider):
    """OpenAI implementation of LLM provider."""

    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = model

    async def generate_response(self, prompt: str, context: Optional[Dict] = None) -> str:
        """Generate a response from OpenAI."""
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3
            )
            return response.choices[0].message.content
        except Exception as e:
            logging.error(f"Error generating response from OpenAI: {e}")
            return f"Error: {str(e)}"

    async def process_intent(self, text: str, available_actions: List[str]) -> RobotCommand:
        """Process text and extract robot command using OpenAI."""
        # Create a prompt to extract intent and parameters
        prompt = f"""
        Given the user command: "{text}"

        Available robot actions: {', '.join(available_actions)}

        Respond in JSON format with the following structure:
        {{
            "action": "the most appropriate action from the list",
            "parameters": {{"param1": "value1", "param2": "value2"}},
            "confidence": 0.0-1.0,
            "intent": "brief description of the intent"
        }}

        Only respond with the JSON, no other text.
        """

        try:
            response_text = await self.generate_response(prompt)

            # Extract JSON from response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                data = json.loads(json_str)

                return RobotCommand(
                    action=data.get('action', ''),
                    parameters=data.get('parameters', {}),
                    confidence=data.get('confidence', 0.5),
                    intent=data.get('intent', '')
                )
        except Exception as e:
            logging.error(f"Error processing intent with OpenAI: {e}")

        # Return default command if processing fails
        return RobotCommand(
            action='unknown',
            parameters={},
            confidence=0.0,
            intent=text
        )


class AnthropicLLM(LLMProvider):
    """Anthropic implementation of LLM provider."""

    def __init__(self, api_key: str, model: str = "claude-3-haiku-20240307"):
        self.client = AsyncAnthropic(api_key=api_key)
        self.model = model

    async def generate_response(self, prompt: str, context: Optional[Dict] = None) -> str:
        """Generate a response from Anthropic."""
        try:
            response = await self.client.messages.create(
                model=self.model,
                max_tokens=1024,
                temperature=0.3,
                messages=[{"role": "user", "content": prompt}]
            )
            return response.content[0].text
        except Exception as e:
            logging.error(f"Error generating response from Anthropic: {e}")
            return f"Error: {str(e)}"

    async def process_intent(self, text: str, available_actions: List[str]) -> RobotCommand:
        """Process text and extract robot command using Anthropic."""
        prompt = f"""
        Given the user command: "{text}"

        Available robot actions: {', '.join(available_actions)}

        Respond in JSON format with the following structure:
        {{
            "action": "the most appropriate action from the list",
            "parameters": {{"param1": "value1", "param2": "value2"}},
            "confidence": 0.0-1.0,
            "intent": "brief description of the intent"
        }}

        Only respond with the JSON, no other text.
        """

        try:
            response_text = await self.generate_response(prompt)

            # Extract JSON from response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                data = json.loads(json_str)

                return RobotCommand(
                    action=data.get('action', ''),
                    parameters=data.get('parameters', {}),
                    confidence=data.get('confidence', 0.5),
                    intent=data.get('intent', '')
                )
        except Exception as e:
            logging.error(f"Error processing intent with Anthropic: {e}")

        # Return default command if processing fails
        return RobotCommand(
            action='unknown',
            parameters={},
            confidence=0.0,
            intent=text
        )


class SpeechToTextProcessor:
    """Handles speech-to-text conversion."""

    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def listen_for_speech(self, timeout: int = 5) -> Optional[str]:
        """Listen for speech and convert to text."""
        try:
            with self.microphone as source:
                print("Listening for speech...")
                audio = self.recognizer.listen(source, timeout=timeout)

            print("Processing speech...")
            # Use Google's speech recognition (free, requires internet)
            text = self.recognizer.recognize_google(audio)
            print(f"Heard: {text}")
            return text
        except sr.WaitTimeoutError:
            print("No speech detected within timeout")
            return None
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Error with speech recognition service: {e}")
            return None


class LLMRobotController:
    """Main class that integrates LLM with robot control."""

    def __init__(self, llm_provider: LLMProvider):
        self.llm_provider = llm_provider
        self.speech_processor = SpeechToTextProcessor()

        # Define available robot actions
        self.available_actions = [
            'move_forward',
            'move_backward',
            'turn_left',
            'turn_right',
            'stop',
            'pick_up',
            'put_down',
            'wave',
            'dance',
            'follow_me',
            'go_to_location',
            'fetch_object',
            'take_picture',
            'greet_user'
        ]

        # Initialize ROS 2 if available
        try:
            rclpy.init(args=None)
            self.ros_initialized = True
            self.robot_node = RobotControlNode()
        except:
            self.ros_initialized = False
            self.robot_node = None
            print("ROS 2 not available, running in simulation mode")

    async def process_voice_command(self, command_text: str) -> RobotCommand:
        """Process a voice command and return a robot command."""
        if not command_text:
            return RobotCommand(action='idle', parameters={}, confidence=0.0, intent='no_command')

        # Process the command with the LLM
        robot_command = await self.llm_provider.process_intent(command_text, self.available_actions)
        return robot_command

    def execute_robot_command(self, command: RobotCommand) -> bool:
        """Execute a robot command."""
        print(f"Executing command: {command.action} with parameters: {command.parameters}")

        # If ROS 2 is available, execute the command on the real robot
        if self.ros_initialized and self.robot_node:
            success = self.robot_node.execute_command(command)
        else:
            # Simulate command execution
            success = self.simulate_command_execution(command)

        return success

    def simulate_command_execution(self, command: RobotCommand) -> bool:
        """Simulate command execution for testing purposes."""
        print(f"SIMULATION: Robot performing {command.action}")
        print(f"Parameters: {command.parameters}")
        print(f"Confidence: {command.confidence}")
        print(f"Intent: {command.intent}")

        # Add simulated delays based on command
        import time
        time.sleep(0.5)  # Simulate processing time

        return True  # Simulate success

    async def run_voice_control_loop(self):
        """Main loop for voice-controlled robot interaction."""
        print("Starting voice control loop. Say 'exit' to quit.")

        while True:
            # Listen for voice command
            voice_input = self.speech_processor.listen_for_speech()

            if not voice_input:
                continue

            if 'exit' in voice_input.lower() or 'quit' in voice_input.lower():
                print("Exiting voice control loop...")
                break

            # Process the command with LLM
            robot_command = await self.process_voice_command(voice_input)

            # Execute the command
            success = self.execute_robot_command(robot_command)

            if success:
                print(f"Command '{robot_command.action}' executed successfully")
            else:
                print(f"Failed to execute command '{robot_command.action}'")


class RobotControlNode(Node):
    """ROS 2 node for controlling the robot."""

    def __init__(self):
        super().__init__('llm_robot_controller')

        # Publisher for robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Any other publishers/subscribers needed for robot control
        self.command_publisher = self.create_publisher(String, 'robot_commands', 10)

        self.get_logger().info('LLM Robot Controller node initialized')

    def execute_command(self, command: RobotCommand) -> bool:
        """Execute robot command via ROS 2."""
        try:
            msg = String()
            msg.data = json.dumps({
                'action': command.action,
                'parameters': command.parameters,
                'confidence': command.confidence,
                'intent': command.intent
            })

            self.command_publisher.publish(msg)

            # For movement commands, send velocity commands
            if command.action in ['move_forward', 'move_backward', 'turn_left', 'turn_right', 'stop']:
                twist_msg = Twist()

                if command.action == 'move_forward':
                    twist_msg.linear.x = 0.2  # m/s
                elif command.action == 'move_backward':
                    twist_msg.linear.x = -0.2
                elif command.action == 'turn_left':
                    twist_msg.angular.z = 0.5  # rad/s
                elif command.action == 'turn_right':
                    twist_msg.angular.z = -0.5
                elif command.action == 'stop':
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0

                self.cmd_vel_publisher.publish(twist_msg)

            self.get_logger().info(f'Published command: {command.action}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')
            return False


def load_api_keys(config_path: str = "config/.env"):
    """Load API keys from configuration file."""
    api_keys = {}

    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            for line in f:
                if '=' in line and not line.strip().startswith('#'):
                    key, value = line.strip().split('=', 1)
                    api_keys[key.strip()] = value.strip()

    return api_keys


async def main():
    """Main function to run the LLM integration framework."""
    print("Initializing LLM Integration Framework for Robotics...")

    # Load API keys
    api_keys = load_api_keys()

    # Initialize LLM provider (default to OpenAI if available)
    llm_provider = None

    if 'OPENAI_API_KEY' in api_keys:
        llm_provider = OpenAILLM(api_keys['OPENAI_API_KEY'])
        print("Using OpenAI provider")
    elif 'ANTHROPIC_API_KEY' in api_keys:
        llm_provider = AnthropicLLM(api_keys['ANTHROPIC_API_KEY'])
        print("Using Anthropic provider")
    else:
        print("No API keys found. Please set up your .env file with API keys.")
        print("Example .env file contents:")
        print("# OpenAI API key")
        print("OPENAI_API_KEY=your_openai_api_key_here")
        print("# Anthropic API key")
        print("ANTHROPIC_API_KEY=your_anthropic_api_key_here")
        return

    # Create the controller
    controller = LLMRobotController(llm_provider)

    # Run the voice control loop
    await controller.run_voice_control_loop()


if __name__ == '__main__':
    asyncio.run(main())
EOF

echo "Created LLM integration framework: $LLM_ROOT/llm_robotics_framework.py"

# Create a requirements file
cat << EOF > requirements.txt
# Core dependencies for LLM Robotics Integration
torch>=2.0.0
torchvision>=0.15.0
torchaudio>=2.0.0
transformers>=4.21.0
openai>=1.0.0
anthropic>=0.5.0
cohere>=4.0.0
sentence-transformers>=2.2.0
numpy>=1.21.0
pandas>=1.3.0
scikit-learn>=1.0.0

# Audio processing
pyaudio>=0.2.11
speechrecognition>=3.8.1
librosa>=0.9.0
sounddevice>=0.4.6

# Web and networking
requests>=2.25.0
aiohttp>=3.8.0
asyncio

# Configuration and utilities
python-dotenv>=0.19.0

# ROS 2 (assumes ROS 2 is already installed and sourced)
rclpy
EOF

echo "Created requirements file: $LLM_ROOT/requirements.txt"

# Create an example configuration file
cat << EOF > config_example.yaml
# LLM Integration Configuration for Robotics

llm_integration:
  # Available providers: openai, anthropic, cohere
  provider: "openai"

  # Model settings
  model: "gpt-3.5-turbo"  # or "claude-3-haiku-20240307" for Anthropic

  # API settings
  api_base: ""  # Leave empty for default, or specify custom endpoint
  timeout: 30   # seconds

  # Prompt engineering settings
  system_prompt: |
    You are a helpful assistant that interprets natural language commands for a humanoid robot.
    The robot can perform actions like moving, manipulating objects, and interacting with people.
    Always respond in the specified JSON format with appropriate actions and parameters.

  # Available robot actions
  available_actions:
    - "move_forward"
    - "move_backward"
    - "turn_left"
    - "turn_right"
    - "stop"
    - "pick_up"
    - "put_down"
    - "wave"
    - "dance"
    - "follow_me"
    - "go_to_location"
    - "fetch_object"
    - "take_picture"
    - "greet_user"

robot_interface:
  # ROS 2 settings
  ros_enabled: true
  command_topic: "/robot_commands"
  velocity_topic: "/cmd_vel"

  # Action execution settings
  action_timeout: 10.0  # seconds
  min_confidence: 0.7   # minimum confidence to execute command

audio_processing:
  # Microphone settings
  microphone_index: -1  # -1 for default, or specific device index
  sample_rate: 16000
  chunk_size: 1024

  # Speech recognition settings
  energy_threshold: 300  # Adjust based on ambient noise
  dynamic_energy_threshold: true
  pause_threshold: 0.8   # seconds of silence to wait for
  phrase_time_limit: 10  # maximum seconds for a phrase

error_handling:
  # Retry settings
  max_retries: 3
  retry_delay: 1.0  # seconds

  # Fallback behaviors
  fallback_action: "idle"
  confidence_threshold: 0.5
EOF

echo "Created example configuration: $LLM_ROOT/config_example.yaml"

# Create documentation
cat << EOF > llm_integration_documentation.md
# LLM Integration Framework for Robotics

## Overview

This framework provides the infrastructure to integrate Large Language Models (LLMs) with robotics systems for voice-driven control and task planning. The framework handles:

- Speech-to-text conversion
- Natural language understanding with LLMs
- Command extraction and validation
- Robot action execution
- Error handling and fallback mechanisms

## Architecture

### Components

1. **LLM Providers**: Abstract base class with implementations for different LLM services (OpenAI, Anthropic, etc.)
2. **Speech Processor**: Handles audio input and conversion to text
3. **Intent Processor**: Converts natural language to robot commands
4. **Robot Controller**: Executes commands on the robot (real or simulated)
5. **Configuration Manager**: Handles API keys and settings

### Supported LLM Providers

- OpenAI (GPT models)
- Anthropic (Claude models)
- Cohere (Command models)

## Setup

1. Create a .env file with your API keys:
   \`\`\`
   OPENAI_API_KEY=your_openai_api_key_here
   ANTHROPIC_API_KEY=your_anthropic_api_key_here
   \`\`\`

2. Install dependencies: \`pip install -r requirements.txt\`

3. Run the framework: \`python llm_robotics_framework.py\`

## Usage

The framework operates in a voice control loop:

1. Listen for voice input
2. Convert speech to text
3. Process text with LLM to extract intent
4. Execute appropriate robot action
5. Repeat

## Configuration

See config_example.yaml for all configurable parameters including:

- LLM provider settings
- Robot interface settings
- Audio processing parameters
- Error handling configurations

## Extending

To add new robot actions:

1. Add the action to the available_actions list
2. Implement the action execution in RobotControlNode
3. Train the LLM with examples of commands that should trigger the action

## Security Considerations

- Store API keys securely, never commit to version control
- Implement rate limiting for API calls
- Validate all commands before execution
- Consider implementing user authentication for sensitive actions

## Troubleshooting

- If speech recognition fails, check microphone permissions and ambient noise levels
- If LLM calls fail, verify API keys and internet connection
- If robot commands don't execute, check ROS 2 setup and topic connections
EOF

echo "Created documentation: $LLM_ROOT/llm_integration_documentation.md"

# Create a simple test script
cat << 'EOF' > test_llm_integration.py
#!/usr/bin/env python3
"""
Simple test script for LLM Integration Framework
"""

import asyncio
import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from llm_robotics_framework import OpenAILLM, RobotCommand, load_api_keys


async def test_llm_integration():
    """Test the LLM integration framework."""
    print("Testing LLM Integration Framework...")

    # Load API keys
    api_keys = load_api_keys()

    if 'OPENAI_API_KEY' not in api_keys:
        print("OpenAI API key not found. Please set OPENAI_API_KEY in your .env file.")
        return False

    # Initialize LLM provider
    llm = OpenAILLM(api_keys['OPENAI_API_KEY'], model="gpt-3.5-turbo")

    # Test basic response generation
    print("\n1. Testing basic response generation...")
    response = await llm.generate_response("Say hello in a friendly way.")
    print(f"Response: {response}")

    # Test intent processing
    print("\n2. Testing intent processing...")
    available_actions = [
        'move_forward', 'move_backward', 'turn_left', 'turn_right',
        'pick_up', 'put_down', 'wave', 'greet_user'
    ]

    test_commands = [
        "Please move forward slowly",
        "Turn to your left",
        "Wave to me",
        "Greet the person in front of you"
    ]

    for command in test_commands:
        robot_cmd = await llm.process_intent(command, available_actions)
        print(f"Input: '{command}'")
        print(f"  Action: {robot_cmd.action}")
        print(f"  Parameters: {robot_cmd.parameters}")
        print(f"  Confidence: {robot_cmd.confidence}")
        print(f"  Intent: {robot_cmd.intent}")
        print()

    print("LLM Integration Framework test completed successfully!")
    return True


if __name__ == '__main__':
    success = asyncio.run(test_llm_integration())
    if not success:
        sys.exit(1)
EOF

echo "Created test script: $LLM_ROOT/test_llm_integration.py"

# Make scripts executable
chmod +x llm_robotics_framework.py
chmod +x test_llm_integration.py

echo ""
echo "========================================="
echo "LLM INTEGRATION FRAMEWORK SETUP COMPLETE!"
echo "========================================="
echo ""
echo "Setup includes:"
echo "- Core framework for LLM integration with robotics ($LLM_ROOT/llm_robotics_framework.py)"
echo "- Requirements file with all dependencies ($LLM_ROOT/requirements.txt)"
echo "- Configuration example ($LLM_ROOT/config_example.yaml)"
echo "- Documentation ($LLM_ROOT/llm_integration_documentation.md)"
echo "- Test script ($LLM_ROOT/test_llm_integration.py)"
echo ""
echo "To use the framework:"
echo "1. Activate the virtual environment: source $LLM_ROOT/llm_env/bin/activate"
echo "2. Set up your API keys in $LLM_ROOT/config/.env"
echo "3. Install dependencies: pip install -r requirements.txt"
echo "4. Run the framework: python llm_robotics_framework.py"
echo "5. Run tests: python test_llm_integration.py"
echo ""