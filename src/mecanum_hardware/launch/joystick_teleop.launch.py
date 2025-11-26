#!/usr/bin/env python3
"""
Launch file for Joystick Teleoperation
Starts joy_node and joystick_teleop together
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='1.0',
            description='Maximum linear velocity in m/s'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='2.0',
            description='Maximum angular velocity in rad/s'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.1',
            description='Joystick deadzone threshold'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'device',
            default_value='/dev/input/js0',
            description='Joystick device path'
        )
    )
    
    # Get arguments
    max_linear_vel = LaunchConfiguration('max_linear_vel')
    max_angular_vel = LaunchConfiguration('max_angular_vel')
    deadzone = LaunchConfiguration('deadzone')
    device = LaunchConfiguration('device')
    
    # Joy Node - reads from joystick hardware
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': device,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )
    
    # Joystick Teleop Node - converts joy to cmd_vel
    joystick_teleop_node = Node(
        package='mecanum_hardware',
        executable='scripts/joystick_teleop.py',  # Include scripts/ prefix
        name='joystick_teleop',
        parameters=[{
            'max_linear_vel': max_linear_vel,
            'max_angular_vel': max_angular_vel,
            'deadzone': deadzone,
        }],
        output='screen',
    )
    
    nodes = [
        joy_node,
        joystick_teleop_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)