#!/usr/bin/env python3
"""
Complete Inventory Mission Launch File

This launches:
1. Hardware layer (controllers, sensors)
2. Navigation stack (Nav2 with AMCL)
3. QR Detection system (enhanced neon detection)
4. Database system (auto-updates from QR detection)
5. Actuator control
6. Waypoint follower (coordinates everything)

Usage:
  ros2 launch mecanum_hardware inventory_mission.launch.py \
      map:=/path/to/map.yaml \
      waypoints:=/path/to/waypoints.yaml
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
    qr_detection_dir = get_package_share_directory('warehouse_rover_qr_detection')
    image_processing_dir = get_package_share_directory('warehouse_rover_image_processing')
    database_dir = get_package_share_directory('warehouse_rover_database')
    
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
            'waypoints',
            default_value='/tmp/waypoints.yaml',
            description='Full path to waypoints YAML file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'scan_duration',
            default_value='50.0',
            description='Duration (seconds) to scan QR codes at each waypoint'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device path'
        )
    )
    
    # ===== 1. HARDWARE LAYER =====
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hw_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={'use_sim': 'false'}.items()
    )
    
    # ===== 2. CARTOGRAPHER (for odometry fusion) =====
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
    
    # ===== 3. NAV2 STACK =====
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
    
    # ===== 4. CAMERA & QR DETECTION =====
    # Camera bridge node
    camera_node = Node(
        package='warehouse_rover_image_processing',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('camera_device'),
            'frame_rate': 30,
            'resolution_width': 640,
            'resolution_height': 480
        }]
    )
    
    # Enhanced QR detector (with neon detection)
    qr_detector_node = Node(
        package='warehouse_rover_image_processing',
        executable='qr_detector_enhanced_node',
        name='qr_detector_enhanced',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'detection_rate': 10.0,
            'neon_enhancement': True
        }]
    )
    
    # ===== 5. DATABASE (auto-updates from QR detection) =====
    database_node = Node(
        package='warehouse_rover_database',
        executable='inventory_node_mongo.py',
        name='inventory_database',
        output='screen',
        parameters=[{
            'use_local_mongodb': True,  # or use Atlas
            'auto_export': True,
            'export_dir': '/tmp/inventory_missions'
        }]
    )
    
    # ===== 6. ACTUATOR CONTROL =====
    actuator_node = Node(
        package='mecanum_hardware',
        executable='actuator_control_node.py',
        name='actuator_control',
        output='screen'
    )
    
    # ===== 7. WAYPOINT FOLLOWER (Main coordinator) =====
    waypoint_follower = Node(
        package='mecanum_hardware',
        executable='waypoint_follower_node.py',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints'),
            'scan_duration': LaunchConfiguration('scan_duration'),
            'actuator_lift_time': 5.0,
            'actuator_lower_time': 5.0
        }]
    )
    
    # ===== LAUNCH SEQUENCE =====
    return LaunchDescription(declared_arguments + [
        # Startup banner
        LogInfo(msg="=" * 70),
        LogInfo(msg="  WAREHOUSE ROVER - INVENTORY MISSION"),
        LogInfo(msg="=" * 70),
        
        # t=0s: Hardware
        LogInfo(msg="[0s]  Starting hardware layer..."),
        hardware_launch,
        
        # t=5s: Sensors + Actuator
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[5s]  Starting sensors and actuator..."),
                actuator_node
            ]
        ),
        
        # t=8s: Cartographer odometry
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="[8s]  Starting Cartographer odometry..."),
                cartographer_node
            ]
        ),
        
        # t=10s: Nav2 stack
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg="[10s] Starting Nav2 navigation stack..."),
                nav2_bringup
            ]
        ),
        
        # t=15s: Camera and QR detection
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg="[15s] Starting camera and QR detection..."),
                camera_node,
                qr_detector_node
            ]
        ),
        
        # t=18s: Database
        TimerAction(
            period=18.0,
            actions=[
                LogInfo(msg="[18s] Starting database system..."),
                database_node
            ]
        ),
        
        # t=25s: Waypoint follower (needs Nav2 to be ready)
        TimerAction(
            period=25.0,
            actions=[
                LogInfo(msg="[25s] Starting waypoint follower..."),
                LogInfo(msg="=" * 70),
                LogInfo(msg="  AUTONOMOUS INVENTORY MISSION STARTING!"),
                LogInfo(msg="=" * 70),
                waypoint_follower
            ]
        ),
        
        # t=30s: System ready message
        TimerAction(
            period=30.0,
            actions=[
                LogInfo(msg=""),
                LogInfo(msg="=" * 70),
                LogInfo(msg="  INVENTORY MISSION SYSTEM READY!"),
                LogInfo(msg="=" * 70),
                LogInfo(msg=""),
                LogInfo(msg="WORKFLOW:"),
                LogInfo(msg="  1. Robot navigates to waypoint"),
                LogInfo(msg="  2. Reaches waypoint -> Lifts actuator (5s)"),
                LogInfo(msg="  3. Scans QR codes (50s default)"),
                LogInfo(msg="  4. Lowers actuator (5s)"),
                LogInfo(msg("  5. Moves to next waypoint")),
                LogInfo(msg=""),
                LogInfo(msg="MONITORING:"),
                LogInfo(msg="  • Status: ros2 topic echo /waypoint_follower/status"),
                LogInfo(msg="  • QR Detections: ros2 topic echo /qr_detections"),
                LogInfo(msg="  • Navigation: RViz"),
                LogInfo(msg=""),
                LogInfo(msg="MANUAL CONTROL (if needed):"),
                LogInfo(msg="  • Actuator: ros2 topic pub /actuator/command std_msgs/String \"data: 'up'\""),
                LogInfo(msg="  • Emergency stop: Ctrl+C"),
                LogInfo(msg="=" * 70)
            ]
        )
    ])
