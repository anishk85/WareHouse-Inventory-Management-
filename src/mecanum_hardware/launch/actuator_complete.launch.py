#!/usr/bin/env python3
"""
Launch file for Actuator Control System with Height Monitoring

This launches:
1. Laser distance sensor node
2. Height monitor node
3. Actuator control node with PID
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial0',
        description='Serial port for laser sensor'
    )
    
    min_height_arg = DeclareLaunchArgument(
        'min_height',
        default_value='0.7',
        description='Minimum safe height in meters'
    )
    
    max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='1.6',
        description='Maximum safe height in meters'
    )
    
    pid_kp_arg = DeclareLaunchArgument(
        'pid_kp',
        default_value='50.0',
        description='PID Proportional gain'
    )
    
    pid_ki_arg = DeclareLaunchArgument(
        'pid_ki',
        default_value='0.1',
        description='PID Integral gain'
    )
    
    pid_kd_arg = DeclareLaunchArgument(
        'pid_kd',
        default_value='5.0',
        description='PID Derivative gain'
    )
    
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.2',
        description='Target distance in meters (within min/max range)'
    )
    
    # Laser sensor node
    laser_node = Node(
        package='mecanum_hardware',  # Replace with your package name
        executable='laser_sensor.py',
        name='laser_sensor',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 9600,
            'frame_id': 'laser_sensor',
            'publish_rate': 20.0,
            'auto_upload_interval': 5
        }]
    )
    
    # Height moniyour_package_nametor node
    height_monitor_node = Node(
        package='mecanum_hardware',  # Replace with your package name
        executable='height_monitor_node.py',
        name='height_monitor_node',
        output='screen',
        parameters=[{
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'warning_margin': 0.05
        }]
    )
    
    # Actuator control node with PID
    actuator_node = Node(
        package='mecanum_hardware',  # Replace with your package name
        executable='actuator_control_node.py',
        name='actuator_control_node',
        output='screen',
        parameters=[{
            'pid_kp': LaunchConfiguration('pid_kp'),
            'pid_ki': LaunchConfiguration('pid_ki'),
            'pid_kd': LaunchConfiguration('pid_kd'),
            'target_distance': LaunchConfiguration('target_distance'),
            'distance_tolerance': 0.02,
            'min_speed': 30,
            'max_speed': 100,
            'deadband': 0.01
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        min_height_arg,
        max_height_arg,
        pid_kp_arg,
        pid_ki_arg,
        pid_kd_arg,
        target_distance_arg,
        laser_node,
        height_monitor_node,
        actuator_node
    ])

