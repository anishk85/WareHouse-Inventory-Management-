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
    
    # YDLIDAR usually mounts on USB0 if it's the only serial device
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB2',
        description='Serial port for YDLIDAR'))

    # MATCHES URDF: lidar_link is defined in mecanum_in_gazebo/urdf/lidar.xacro
    # The URDF creates: base_link → lidar_link (fixed joint at 0.08m height)
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_frame', default_value='lidar_link',
        description='Frame ID for LiDAR scan messages (must match URDF lidar.xacro)'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_frequency', default_value='10.0',
        description='Scan frequency in Hz (lower = less power)'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_sample_rate', default_value='3',
        description='Sample rate (3=3K samples/sec)'))

    # ================= 2. PATHS =================
    pkg_nav = 'navigation_setup'

    # Cartographer Configuration
    cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare(pkg_nav), 'config'])
    cartographer_config_file = 'cartographer_mapping.lua'

    # ================= 3. SENSOR NODES =================
    
    # --- YDLIDAR X2 NODE (direct, no lifecycle wrapper) ---
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('lidar_port'),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'ignore_array': '',
            'baudrate': 115200,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': LaunchConfiguration('lidar_sample_rate'),
            'abnormal_check_count': 4,
            'resolution_fixed': True,
            'reversion': False,
            'inverted': False,
            'isSingleChannel': True,
            'intensity': False,
            'support_motor_dtr': False,
            'angle_min': -180.0,
            'angle_max': 180.0,
            'range_min': 0.1,
            'range_max': 12.0,
            'frequency': LaunchConfiguration('lidar_frequency'),
            'connection_delay': 1000,  
            'intensity': False,
            'qos_overrides./scan.publisher.reliability': 'reliable',
        }],
        remappings=[
            ('scan', '/scan')  # Remap to /scan for Cartographer
        ]
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
        LogInfo(msg="---> Starting YDLIDAR and SLAM..."),
        ydlidar_node,
        imu_node,
        delayed_cartographer
    ])