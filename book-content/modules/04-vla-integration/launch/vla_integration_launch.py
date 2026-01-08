from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Voice-to-Text Converter Node
        Node(
            package='your_robot_package',
            executable='voice_to_text_converter',
            name='voice_to_text_converter',
            output='screen',
            parameters=[
                {'language': 'en-US'},
                {'timeout': 5},
                {'phrase_time_limit': 10},
                {'energy_threshold': 300}
            ]
        ),

        # Voice Command Processor Node
        Node(
            package='your_robot_package',
            executable='voice_command_processor',
            name='voice_command_processor',
            output='screen'
        ),

        # Intent Recognizer Node
        Node(
            package='your_robot_package',
            executable='intent_recognizer',
            name='intent_recognizer',
            output='screen',
            parameters=[
                {'confidence_threshold': 0.7},
                {'enable_contextual_understanding': True},
                {'entity_extraction_enabled': True}
            ]
        ),

        # Voice-to-Action Pipeline Node
        Node(
            package='your_robot_package',
            executable='voice_to_action_pipeline',
            name='voice_to_action_pipeline',
            output='screen',
            parameters=[
                {'default_linear_speed': 0.5},
                {'default_angular_speed': 0.5},
                {'default_movement_duration': 2.0},
                {'action_timeout': 30.0}
            ]
        ),

        # Action Execution System Node
        Node(
            package='your_robot_package',
            executable='action_execution_system',
            name='action_execution_system',
            output='screen',
            parameters=[
                {'default_linear_speed': 0.5},
                {'default_angular_speed': 0.5},
                {'default_movement_duration': 2.0},
                {'action_timeout': 30.0},
                {'max_queue_size': 10},
                {'concurrent_actions_limit': 3}
            ]
        ),

        # Test Nodes
        Node(
            package='your_robot_package',
            executable='test_voice_command_execution',
            name='voice_command_validator',
            output='screen'
        ),
    ])