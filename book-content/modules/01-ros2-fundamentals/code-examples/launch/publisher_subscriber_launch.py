from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='book_examples',
            executable='talker',
            name='publisher_member_function',
            output='screen'
        ),
        Node(
            package='book_examples',
            executable='listener',
            name='subscriber_member_function',
            output='screen'
        )
    ])