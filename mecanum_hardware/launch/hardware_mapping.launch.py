#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ================= 1. ARGUMENTS =================
    declared_arguments = []
    
    declared_arguments.append(DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Use simulation (Gazebo) or real hardware'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB2',
        description='Serial port for RPLidar'))

    declared_arguments.append(DeclareLaunchArgument(
        'lidar_frame', default_value='lidar_link',
        description='Frame ID for LiDAR scan messages'))

    # ================= 2. PATHS =================
    pkg_nav = 'navigation_setup'

    # Cartographer Configuration
    cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare(pkg_nav), 'config'])
    cartographer_config_file = 'cartographer_mapping.lua'

    # ================= 3. SENSOR NODES =================
    
    # RPLidar Node
    # Note: TF (base_link -> lidar_link) is provided by 'robot_state_publisher' in Terminal 1 launch hardware.launch.py in terminal 1
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

    # IMU Node
    imu_node = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen'
    )

    # ================= 4. SLAM NODES =================

    # Cartographer Node
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
            # CRITICAL: Remap to the topic published by Terminal 1 needed because cartographer takes in input as this 
            ('odom', '/mecanum_drive_controller/odometry'), 
            ('imu', '/imu/data')
        ]
    )

    # Occupancy Grid Node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    
    delayed_cartographer = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="---> Sensors assumed ready. Starting Cartographer..."),
            cartographer_node,
            occupancy_grid_node
        ]
    )

    return LaunchDescription(declared_arguments + [
        LogInfo(msg="---> Starting SENSORS and SLAM (Assuming Hardware is running in another terminal)..."),
        rplidar_node,
        imu_node,
        delayed_cartographer
    ])