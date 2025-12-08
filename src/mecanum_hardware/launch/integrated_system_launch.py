#!/usr/bin/env python3
"""
Launch file for integrated laser sensor and PID actuator control system
Starts both the laser distance sensor and actuator control with PID
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Laser sensor parameters
        DeclareLaunchArgument(
            'laser_serial_port',
            default_value='/dev/serial0',
            description='Serial port for KL200 laser sensor'
        ),
        
        DeclareLaunchArgument(
            'laser_baud_rate',
            default_value='9600',
            description='Baud rate for laser sensor'
        ),
        
        DeclareLaunchArgument(
            'laser_publish_rate',
            default_value='20.0',
            description='Laser publishing rate in Hz'
        ),
        
        # PID parameters
        DeclareLaunchArgument(
            'pid_kp',
            default_value='50.0',
            description='PID Proportional gain'
        ),
        
        DeclareLaunchArgument(
            'pid_ki',
            default_value='0.1',
            description='PID Integral gain'
        ),
        
        DeclareLaunchArgument(
            'pid_kd',
            default_value='5.0',
            description='PID Derivative gain'
        ),
        
        # Target position parameters
        DeclareLaunchArgument(
            'target_distance',
            default_value='1.80',
            description='Target distance in meters (1.80m = 180cm)'
        ),
        
        DeclareLaunchArgument(
            'distance_tolerance',
            default_value='0.02',
            description='Distance tolerance in meters (±2cm)'
        ),
        
        DeclareLaunchArgument(
            'deadband',
            default_value='0.01',
            description='Control deadband in meters (±1cm)'
        ),
        
        # Motor speed parameters
        DeclareLaunchArgument(
            'min_speed',
            default_value='30',
            description='Minimum motor speed (0-100%)'
        ),
        
        DeclareLaunchArgument(
            'max_speed',
            default_value='100',
            description='Maximum motor speed (0-100%)'
        ),
        
        # Launch KL200 laser sensor node
        Node(
            package='mecanum_hardware',
            executable='laser_node',
            name='kl200_laser_sensor',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('laser_serial_port'),
                'baud_rate': LaunchConfiguration('laser_baud_rate'),
                'frame_id': 'laser_sensor',
                'publish_rate': LaunchConfiguration('laser_publish_rate'),
                'auto_upload_interval': 5,  # 500ms
            }],
            remappings=[
                # Laser publishes on /laser_distance which actuator subscribes to
            ]
        ),
        
        # Launch actuator control node with PID
        Node(
            package='mecanum_hardware',  # Replace with your package name
            executable='actuator_pid_control_node',
            name='actuator_control',
            output='screen',
            parameters=[{
                'pid_kp': LaunchConfiguration('pid_kp'),
                'pid_ki': LaunchConfiguration('pid_ki'),
                'pid_kd': LaunchConfiguration('pid_kd'),
                'target_distance': LaunchConfiguration('target_distance'),
                'distance_tolerance': LaunchConfiguration('distance_tolerance'),
                'deadband': LaunchConfiguration('deadband'),
                'min_speed': LaunchConfiguration('min_speed'),
                'max_speed': LaunchConfiguration('max_speed'),
            }]
        )
    ])