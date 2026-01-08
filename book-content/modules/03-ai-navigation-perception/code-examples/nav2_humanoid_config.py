#!/usr/bin/env python3
"""
Nav2 Configuration for Humanoid Robots in the Physical AI & Humanoid Robotics Book

This configuration file sets up the Nav2 stack for humanoid robot navigation,
including path planning, obstacle avoidance, and localization for legged robots.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the package share directory
    package_name = 'book_examples'

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Create the launch description
    ld = LaunchDescription()

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_name, 'config', 'nav2_humanoid_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_bt_xml_file_arg = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(package_name, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use'
    )

    declare_map_subscribe_transient_local_arg = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='False',
        description='Whether to set the map subscriber QoS to transient local'
    )

    # Add launch arguments to the description
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_autostart_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_bt_xml_file_arg)
    ld.add_action(declare_map_subscribe_transient_local_arg)

    # Create the lifecycle manager node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['map_server',
                                  'planner_server',
                                  'controller_server',
                                  'behavior_server',
                                  'bt_navigator',
                                  'waypoint_follower',
                                  'velocity_smoother']}]
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Planner server (with humanoid-specific configuration)
    planner_server = Node(
        package='nav2_navfn_planner',
        executable='navfn_planner',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('goal_checker', 'goal_checker'),
                   ('global_costmap/global_costmap', 'global_costmap'),
                   ('global_costmap/footprint', 'local_costmap/footprint')]
    )

    # Controller server (with humanoid-specific configuration)
    controller_server = Node(
        package='nav2_dwb_controller',
        executable='dwb_controller',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel'),
                   ('global_plan', 'global_plan'),
                   ('local_plan', 'local_plan'),
                   ('feedback', 'dwb_feedback'),
                   ('trajectory', 'trajectory'),
                   ('waypoints', 'waypoints')]
    )

    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel'),
                   ('global_costmap', 'global_costmap'),
                   ('local_costmap', 'local_costmap'),
                   ('acc_lim_x', 'acc_lim_x'),
                   ('acc_lim_theta', 'acc_lim_theta')]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time},
                   {'bt_xml_filename': bt_xml_file}],
        remappings=[('local_costmap/local_costmap', 'local_costmap'),
                   ('global_costmap/global_costmap', 'global_costmap'),
                   ('acceleration_limit', 'acc_lim_x')]
    )

    # Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('goal_checker', 'goal_checker')]
    )

    # Velocity smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_output', 'cmd_vel_smoothed'),
                   ('cmd_vel_input', 'cmd_vel')]
    )

    # Add all nodes to the launch description
    ld.add_action(lifecycle_manager)
    ld.add_action(map_server)
    ld.add_action(planner_server)
    ld.add_action(controller_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)

    return ld