#!/usr/bin/env python3
"""
Waypoint Creation Mode Launch File

This mode allows you to:
1. Navigate the robot with joystick/teleop
2. Press a button to save current position as waypoint
3. Waypoints are saved to a YAML file for later autonomous navigation

Usage:
  ros2 launch mecanum_hardware waypoint_creation.launch.py \
      map:=/path/to/map.yaml \
      waypoints_output:=/path/to/save/waypoints.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    mecanum_hw_dir = get_package_share_directory('mecanum_hardware')
    navigation_setup_dir = get_package_share_directory('navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params = os.path.join(navigation_setup_dir, 'config', 'nav2_params.yaml')
    cartographer_config_dir = os.path.join(navigation_setup_dir, 'config')
    
    # Launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(navigation_setup_dir, 'maps', 'warehouse.yaml'),
            description='Full path to map YAML file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'waypoints_output',
            default_value='/tmp/waypoints.yaml',
            description='Output file for saved waypoints'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'joy_device',
            default_value='/dev/input/js0',
            description='Joystick device'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'save_button',
            default_value='0',
            description='Button number to save waypoint (0=A on Xbox controller)'
        )
    )
    
    # ===== 1. HARDWARE LAYER =====
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hw_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={'use_sim': 'false'}.items()
    )
    
    # ===== 2. CARTOGRAPHER (for odometry) =====
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_odom.lua'
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/mecanum_drive_controller/odometry'),
            ('imu', '/imu/data')
        ]
    )
    
    # ===== 3. NAV2 STACK (for localization) =====
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )
    
    # ===== 4. JOYSTICK DRIVER =====
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'dev': LaunchConfiguration('joy_device')
        }]
    )
    
    # ===== 5. ENHANCED TELEOP WITH WAYPOINT SAVING =====
    teleop_node = Node(
        package='mecanum_hardware',
        executable='teleop_with_waypoint_save.py',
        name='teleop_with_waypoint_save',
        output='screen',
        parameters=[{
            'linear_scale': 0.5,
            'angular_scale': 1.0,
            'waypoints_file': LaunchConfiguration('waypoints_output'),
            'save_button': LaunchConfiguration('save_button'),
            'use_odom': False  # Use TF from AMCL for better accuracy
        }]
    )
    
    # ===== LAUNCH SEQUENCE =====
    return LaunchDescription(declared_arguments + [
        # Startup banner
        LogInfo(msg="=" * 70),
        LogInfo(msg="  WAYPOINT CREATION MODE"),
        LogInfo(msg="=" * 70),
        
        # t=0s: Hardware
        LogInfo(msg="[0s]  Starting hardware layer..."),
        hardware_launch,
        
        # t=5s: Cartographer
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[5s]  Starting Cartographer odometry..."),
                cartographer_node
            ]
        ),
        
        # t=8s: Nav2
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="[8s]  Starting Nav2 for localization..."),
                nav2_bringup
            ]
        ),
        
        # t=15s: Joystick and teleop
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg="[15s] Starting joystick and teleop..."),
                joy_node,
                teleop_node
            ]
        ),
        
        # t=20s: Ready message
        TimerAction(
            period=20.0,
            actions=[
                LogInfo(msg(""),
                LogInfo(msg="=" * 70),
                LogInfo(msg="  WAYPOINT CREATION MODE READY!"),
                LogInfo(msg="=" * 70),
                LogInfo(msg=""),
                LogInfo(msg="INSTRUCTIONS:"),
                LogInfo(msg="  1. Set initial pose in RViz (2D Pose Estimate)"),
                LogInfo(msg="  2. Drive robot to desired locations using joystick"),
                LogInfo(msg="  3. Press A button (or configured button) to save waypoint"),
                LogInfo(msg="  4. Repeat for all desired waypoints"),
                LogInfo(msg=""),
                LogInfo(msg="JOYSTICK CONTROLS:"),
                LogInfo(msg="  • Left Stick Y: Move forward/backward"),
                LogInfo(msg="  • Right Stick X: Rotate left/right"),
                LogInfo(msg="  • A Button: Save current position as waypoint"),
                LogInfo(msg=""),
                LogInfo(msg="OUTPUT FILE:"),
                LogInfo(msg="  Waypoints will be saved to: $(var waypoints_output)"),
                LogInfo(msg=""),
                LogInfo(msg="NEXT STEPS:"),
                LogInfo(msg="  After collecting waypoints, run inventory mission:"),
                LogInfo(msg="  ros2 launch mecanum_hardware inventory_mission.launch.py \\"),
                LogInfo(msg="      waypoints:=$(var waypoints_output)"),
                LogInfo(msg="=" * 70)
            ]
        )
    ])
