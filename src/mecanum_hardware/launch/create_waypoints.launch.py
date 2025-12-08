#!/usr/bin/env python3
"""
Waypoint Creation Mode
Use joystick to drive around and save waypoints at racks

Controls:
  - Drive with joystick
  - Press Button A at racks to save waypoint
  - Waypoints are saved automatically

After creating waypoints, run autonomous_inventory_mission.launch.py
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
    lidar_filter_config = os.path.join(mecanum_hw_dir, 'config', 'box_filters.yaml')
    cartographer_config_dir = os.path.join(navigation_setup_dir, 'config')
    nav2_params = os.path.join(navigation_setup_dir, 'config', 'nav2_params.yaml')
    
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(navigation_setup_dir, 'maps', 'warehouse_map.yaml'),
            description='Full path to map YAML file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB2',
            description='Serial port for RPLidar'
        )
    )
    
    # Hardware
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hw_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={'use_sim': 'false'}.items()
    )
    
    # Sensors
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
        }],
        remappings=[('scan', '/scan_raw')]
    )
    
    lidar_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[lidar_filter_config],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')]
    )
    
    imu_node = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen'
    )
    
    # Navigation
    cartographer_odom_node = Node(
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
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
            'autostart': 'true',
        }.items()
    )
    
    # Waypoint Manager
    waypoint_manager_node = Node(
        package='mecanum_hardware',
        executable='waypoint_manager_node.py',
        name='waypoint_manager',
        output='screen',
        parameters=[{
            'waypoints_file': '/tmp/warehouse_waypoints.json',
            'auto_save': True
        }]
    )
    
    # Enhanced Joystick Teleop
    joystick_teleop_node = Node(
        package='mecanum_hardware',
        executable='enhanced_joystick_teleop.py',
        name='enhanced_joystick_teleop',
        output='screen',
        parameters=[{
            'max_linear_speed': 0.3,  # Slower for precise positioning
            'max_angular_speed': 0.5,
            'deadzone': 0.1
        }]
    )
    
    # Joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    return LaunchDescription(declared_arguments + [
        LogInfo(msg="=" * 70),
        LogInfo(msg="    WAYPOINT CREATION MODE"),
        LogInfo(msg="=" * 70),
        LogInfo(msg=""),
        LogInfo(msg="Use joystick to drive and save waypoints:"),
        LogInfo(msg="  • Drive with left stick"),
        LogInfo(msg="  • Strafe with right stick"),
        LogInfo(msg="  • Press Button A at RACKS to save waypoint"),
        LogInfo(msg="  • Press Button B at STATIONS to save waypoint"),
        LogInfo(msg("  • Press Button X at CHECKPOINTS to save waypoint")),
        LogInfo(msg=""),
        LogInfo(msg="=" * 70),
        
        # t=0s: Hardware
        hardware_launch,
        
        # t=5s: Sensors
        TimerAction(
            period=5.0,
            actions=[
                rplidar_node,
                lidar_filter_node,
                imu_node
            ]
        ),
        
        # t=8s: Navigation
        TimerAction(
            period=8.0,
            actions=[cartographer_odom_node]
        ),
        
        # t=12s: Nav2
        TimerAction(
            period=12.0,
            actions=[nav2_bringup]
        ),
        
        # t=15s: Waypoint manager + Joystick
        TimerAction(
            period=15.0,
            actions=[
                waypoint_manager_node,
                joy_node,
                joystick_teleop_node
            ]
        ),
        
        # t=20s: Ready
        TimerAction(
            period=20.0,
            actions=[
                LogInfo(msg=""),
                LogInfo(msg="=" * 70),
                LogInfo(msg="    READY TO CREATE WAYPOINTS!"),
                LogInfo(msg="=" * 70),
                LogInfo(msg=""),
                LogInfo(msg("1. Set initial pose in RViz (2D Pose Estimate)")),
                LogInfo(msg="2. Drive to each rack"),
                LogInfo(msg="3. Position robot in front of rack"),
                LogInfo(msg("4. Press Button A to save RACK waypoint")),
                LogInfo(msg=""),
                LogInfo(msg="View saved waypoints:"),
                LogInfo(msg="  cat /tmp/warehouse_waypoints.json"),
                LogInfo(msg=""),
                LogInfo(msg="When done, run autonomous mission:"),
                LogInfo(msg="  ros2 launch mecanum_hardware autonomous_inventory_mission.launch.py"),
                LogInfo(msg("=" * 70))
            ]
        )
    ])
