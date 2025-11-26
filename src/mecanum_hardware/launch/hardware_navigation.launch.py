#!/usr/bin/env python3
"""
Navigation Launch - Nav2 with AMCL + Cartographer sensor fusion
Assumes hardware.launch.py is already running in Terminal 1

Terminal 1: ros2 launch mecanum_hardware hardware.launch.py
Terminal 2: ros2 launch navigation_setup navigation.launch.py

TF chain during navigation:
  map -> odom (AMCL localization)
  odom -> base_link (Cartographer with IMU+wheel odometry fusion)
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
    # ================= 1. PACKAGE DIRECTORIES =================
    navigation_setup_dir = get_package_share_directory('navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ================= 2. CONFIGURATION FILES =================
    nav2_params = os.path.join(navigation_setup_dir, 'config', 'nav2_params.yaml')
    cartographer_config_dir = os.path.join(navigation_setup_dir, 'config')
    cartographer_config_file = 'cartographer_odom.lua'
    
    # ================= 3. LAUNCH ARGUMENTS =================
    declared_arguments = []
    
    declared_arguments.append(DeclareLaunchArgument(
        'map',
        default_value=os.path.join(navigation_setup_dir, 'maps', 'maps.yaml'),
        description='Full path to the map YAML file'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to Nav2 params file'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB2',
        description='Serial port for RPLidar'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar_link',
        description='Frame ID for LiDAR scan messages'
    ))
    
    # ================= 4. SENSOR NODES =================
    
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': LaunchConfiguration('lidar_frame'),
            'inverted': False,
            'angle_compensate': True,
        }]
    )
    
    imu_node = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen',
        respawn=False  # Don't respawn to avoid double shutdown
    )
    
    # ================= 5. CARTOGRAPHER AS ODOMETRY =================
    
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
            ('odom', '/mecanum_drive_controller/odometry'),
            ('imu', '/imu/data')
        ]
    )
    
    # ================= 6. NAV2 BRINGUP (COMPLETE STACK) =================
    # This includes lifecycle manager which handles autostart
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': LaunchConfiguration('params_file'),
            'map': LaunchConfiguration('map'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )
    
    # ================= 7. BUILD LAUNCH DESCRIPTION =================
    
    return LaunchDescription(declared_arguments + [
        
        # === Startup Info ===
        LogInfo(msg="========================================"),
        LogInfo(msg=" NAVIGATION MODE - MECANUM ROBOT"),
        LogInfo(msg="========================================"),
        LogInfo(msg="Prerequisites: Terminal 1 running hardware.launch.py"),
        LogInfo(msg=""),
        LogInfo(msg="TF Chain:"),
        LogInfo(msg="  map → odom (AMCL localization)"),
        LogInfo(msg="  odom → base_link (Cartographer + IMU fusion)"),
        LogInfo(msg="  base_link → sensors (robot_state_publisher)"),
        LogInfo(msg="========================================"),
        
        # === t=0s: Start Sensors ===
        LogInfo(msg="[0s]  Starting sensors (LiDAR + IMU)..."),
        rplidar_node,
        imu_node,
        
        # === t=3s: Start Cartographer Odometry ===
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="[3s]  Starting Cartographer odometry fusion..."),
                cartographer_node
            ]
        ),
        
        # === t=7s: Start Complete Nav2 Stack ===
        TimerAction(
            period=7.0,
            actions=[
                LogInfo(msg="[7s]   Starting Nav2 stack (AMCL + Navigation)..."),
                nav2_bringup
            ]
        ),
        
        # === t=15s: Ready Message ===
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg=""),
                LogInfo(msg="========================================"),
                LogInfo(msg=" NAVIGATION SYSTEM READY!"),
                LogInfo(msg="========================================"),
                LogInfo(msg=""),
                LogInfo(msg=" STEP 1: SET INITIAL POSE"),
                LogInfo(msg="   In RViz on your laptop:"),
                LogInfo(msg="   1. Click '2D Pose Estimate' button"),
                LogInfo(msg="   2. Click on map where robot is located"),
                LogInfo(msg="   3. Drag to set orientation (arrow direction)"),
                LogInfo(msg="   4. Watch particle cloud converge (green dots)"),
                LogInfo(msg=""),
                LogInfo(msg="STEP 2: SEND NAVIGATION GOAL"),
                LogInfo(msg="   1. Click 'Nav2 Goal' button"),
                LogInfo(msg="   2. Click destination on map"),
                LogInfo(msg="   3. Drag to set final orientation"),
                LogInfo(msg="   4. Robot will plan path and navigate!"),
                LogInfo(msg=""),
                LogInfo(msg="MONITORING:"),
                LogInfo(msg="   • Green line = Global path (A* planner)"),
                LogInfo(msg="   • Red/Orange = Local path (DWB controller)"),
                LogInfo(msg="   • Blue = Costmap (obstacles + inflation)"),
                LogInfo(msg="   • Green dots = AMCL particle cloud"),
                LogInfo(msg=""),
                LogInfo(msg="TIP: Run RViz on laptop with:"),
                LogInfo(msg="   export ROS_DOMAIN_ID=42  # Match Pi's domain"),
                LogInfo(msg="   ros2 launch nav2_bringup rviz_launch.py"),
                LogInfo(msg="========================================")
            ]
        ),
    ])