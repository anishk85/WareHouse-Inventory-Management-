#!/usr/bin/env python3
"""
Hardware Navigation Launch - Nav2 with AMCL + Cartographer sensor fusion
Uses pre-built map for localization

TF chain during navigation:
  map -> odom (AMCL localization)
  odom -> base_link (Cartographer with IMU+wheel odometry fusion)

This provides the best of both worlds:
- AMCL for global localization on the map
- Cartographer for smooth local odometry with IMU correction
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
    mecanum_hardware_dir = get_package_share_directory('mecanum_hardware')
    navigation_setup_dir = get_package_share_directory('navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params = os.path.join(navigation_setup_dir, 'config', 'nav2_params.yaml')
    cartographer_config_dir = os.path.join(navigation_setup_dir, 'config')
    cartographer_config_file = 'cartographer_odom.lua'  # Odometry mode for sensor fusion
    
    # Launch arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(navigation_setup_dir, 'maps', 'my_map.yaml'),
        description='Path to the map file'
    )
    
    use_depth_camera_arg = DeclareLaunchArgument(
        'use_depth_camera',
        default_value='false',
        description='Enable depth camera'
    )
    
    esp1_port_arg = DeclareLaunchArgument(
        'esp1_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP1'
    )
    
    esp2_port_arg = DeclareLaunchArgument(
        'esp2_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for ESP2'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB2',
        description='Serial port for LiDAR'
    )
    
    # ========== 1. HARDWARE + SENSORS (t=0s) ==========
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hardware_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'use_depth_camera': LaunchConfiguration('use_depth_camera'),
            'esp1_port': LaunchConfiguration('esp1_port'),
            'esp2_port': LaunchConfiguration('esp2_port'),
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items()
    )
    
    # ========== 2. CARTOGRAPHER AS ODOMETRY (t=8s) ==========
    # Publishes odom->base_link with IMU+wheel fusion
    # This provides smooth, drift-corrected odometry for Nav2
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_file
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/mecanum_drive_controller/odom'),
            ('imu', '/imu/data')
        ]
    )
    
    # ========== 3. NAV2 LOCALIZATION (AMCL + Map Server) (t=12s) ==========
    # Publishes map->odom transform based on particle filter localization
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
            'autostart': 'true'
        }.items()
    )
    
    # ========== 4. NAV2 NAVIGATION STACK (t=15s) ==========
    # Path planning, obstacle avoidance, behavior tree
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true'
        }.items()
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(map_yaml_arg)
    ld.add_action(use_depth_camera_arg)
    ld.add_action(esp1_port_arg)
    ld.add_action(esp2_port_arg)
    ld.add_action(lidar_port_arg)
    
    # Info
    ld.add_action(LogInfo(msg="========================================"))
    ld.add_action(LogInfo(msg="HARDWARE NAVIGATION MODE"))
    ld.add_action(LogInfo(msg="AMCL Localization + Cartographer Odometry"))
    ld.add_action(LogInfo(msg="TF Chain:"))
    ld.add_action(LogInfo(msg="  map -> odom (AMCL)"))
    ld.add_action(LogInfo(msg="  odom -> base_link (Cartographer+IMU)"))
    ld.add_action(LogInfo(msg="========================================"))
    
    # Launch sequence
    ld.add_action(hardware_launch)  # t=0s: Hardware + sensors
    
    ld.add_action(TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[8s] Starting Cartographer odometry fusion..."),
            cartographer_node
        ]
    ))
    
    ld.add_action(TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[12s] Starting AMCL localization..."),
            nav2_localization
        ]
    ))
    
    ld.add_action(TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="[15s] Starting Nav2 navigation stack..."),
            nav2_navigation
        ]
    ))
    
    ld.add_action(TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg="========================================"),
            LogInfo(msg="Navigation ready!"),
            LogInfo(msg=""),
            LogInfo(msg="1. Set initial pose on laptop RViz:"),
            LogInfo(msg="   - Click '2D Pose Estimate'"),
            LogInfo(msg="   - Click and drag on map"),
            LogInfo(msg=""),
            LogInfo(msg="2. Send navigation goal:"),
            LogInfo(msg="   - Click 'Nav2 Goal'"),
            LogInfo(msg="   - Click destination on map"),
            LogInfo(msg=""),
            LogInfo(msg="View on laptop RViz (see networking guide)"),
            LogInfo(msg="========================================")
        ]
    ))
    
    return ld
