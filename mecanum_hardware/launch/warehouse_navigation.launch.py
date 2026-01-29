#!/usr/bin/env python3
"""
Complete Warehouse Navigation System Launch File

Components:
1. Navigation Coordinator (waypoint management + lift control)
2. Actuator Control (with height monitoring)
3. Lift Safety (optical flow on RPi4 via ROS_DOMAIN_ID)
4. Optional: Depth estimation safety

RPi4 (via network): Optical Flow + optional Depth Estimation
RPi5 (main): Navigation, Actuator Control, Height Monitoring
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==================== ARGUMENTS ====================
    declared_arguments = []
    
    # Navigation mode
    declared_arguments.append(DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Path to waypoints JSON file (leave empty for manual RViz mode)'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'use_manual_goals',
        default_value='false',
        description='Use manual goals from RViz instead of waypoint file'))
    
    # Height configuration
    declared_arguments.append(DeclareLaunchArgument(
        'min_height',
        default_value='0.7',
        description='Minimum lift height in meters'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'max_height',
        default_value='1.6',
        description='Maximum lift height in meters'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'start_with_max_height',
        default_value='true',
        description='Start zigzag pattern with max height'))
    
    # Safety configuration
    declared_arguments.append(DeclareLaunchArgument(
        'collision_retry_delay',
        default_value='10.0',
        description='Delay in seconds before resuming after collision'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'enable_depth_safety',
        default_value='false',
        description='Enable depth estimation safety (resource intensive)'))
    
    # Laser sensor configuration
    declared_arguments.append(DeclareLaunchArgument(
        'laser_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for KL200 laser sensor'))
    
    # Get configurations
    waypoint_file = LaunchConfiguration('waypoint_file')
    use_manual_goals = LaunchConfiguration('use_manual_goals')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')
    start_with_max_height = LaunchConfiguration('start_with_max_height')
    collision_retry_delay = LaunchConfiguration('collision_retry_delay')
    enable_depth_safety = LaunchConfiguration('enable_depth_safety')
    laser_port = LaunchConfiguration('laser_port')
    
    # ==================== NODES ====================
    
    # 1. KL200 Laser Distance Sensor (for height monitoring)
    laser_sensor_node = Node(
        package='your_package',  # Replace with your package name
        executable='laser_sensor.py',
        name='laser_sensor',
        output='screen',
        parameters=[{
            'serial_port': laser_port,
            'baud_rate': 9600,
            'frame_id': 'laser_sensor',
            'publish_rate': 20.0,
            'auto_upload_interval': 5
        }]
    )
    
    # 2. Height Monitor (safety limits)
    height_monitor_node = Node(
        package='mecanum_hardware',  # Replace with your package name
        executable='height_monitor_node.py',
        name='height_monitor',
        output='screen',
        parameters=[{
            'min_height': min_height,
            'max_height': max_height,
            'warning_margin': 0.05
        }]
    )
    
    # 3. Actuator Control with PID
    actuator_control_node = Node(
        package='mecanum_hardware',  # Replace with your package name
        executable='actuator_control_node.py',
        name='actuator_control',
        output='screen',
        parameters=[{
            'pid_kp': 50.0,
            'pid_ki': 0.1,
            'pid_kd': 5.0,
            'target_distance': 1.2,  # Will be overridden by navigation coordinator
            'distance_tolerance': 0.02,
            'min_speed': 30,
            'max_speed': 100,
            'deadband': 0.01
        }]
    )
    
    # 4. Navigation Coordinator (main logic)
    navigation_coordinator_node = Node(
        package='mecanum_hardware',  # Replace with your package name
        executable='warehouse_navigation_coordinator.py',
        name='navigation_coordinator',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'use_manual_goals': use_manual_goals,
            'min_height': min_height,
            'max_height': max_height,
            'height_tolerance': 0.05,
            'collision_retry_delay': collision_retry_delay,
            'start_with_max_height': start_with_max_height
        }]
    )
    
    # 5. Optical Flow Safety (NOTE: This runs on RPi4 via ROS_DOMAIN_ID)
    # This is here for reference - launch separately on RPi4
    # optical_flow_node = Node(
    #     package='lift_safety',
    #     executable='optical_flow_node',
    #     name='optical_flow_safety',
    #     output='screen',
    #     parameters=[{
    #         'video_topic': '/camera/image_raw',
    #         'frame_width': 320,
    #         'stop_topic': '/lift/stop',
    #         'magnitude_threshold': 12.0,
    #         'min_area': 800,
    #         'angle_tolerance': 25,
    #         'show_debug_windows': False  # Disable on headless RPi4
    #     }],
    #     # NOTE: Launch this separately on RPi4 with proper ROS_DOMAIN_ID
    #     condition=IfCondition('false')  # Disabled by default (run on RPi4)
    # )
    
    # 6. Depth Estimation Safety (OPTIONAL - resource intensive)
    # depth_perception_node = Node(
    #     package='lift_safety',
    #     executable='depth_perception_node',
    #     name='depth_perception_safety',
    #     output='screen',
    #     parameters=[{
    #         'video_topic': '/camera/image_raw',
    #         'stop_topic': '/lift/stop',
    #         'model_id': 'depth-anything/Depth-Anything-V2-Small-hf',
    #         'process_width': 480,
    #         'process_height': 360,
    #         'collision_threshold': 230,
    #         'roi_size': 300,
    #         'show_debug_windows': False
    #     }],
    #     condition=IfCondition(enable_depth_safety)
    # )
    # 
    # ==================== LAUNCH DESCRIPTION ====================
    
    nodes = [
        laser_sensor_node,
        height_monitor_node,
        actuator_control_node,
        navigation_coordinator_node,
        # optical_flow_node,  # Launch separately on RPi4
        # depth_perception_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)