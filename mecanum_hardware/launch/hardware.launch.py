#!/usr/bin/env python3
"""
Hardware launch file for mecanum robot on Raspberry Pi 5
Launches the real robot hardware interface with sensors
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    mecanum_hardware_dir = get_package_share_directory('mecanum_hardware')
    mecanum_gazebo_dir = get_package_share_directory('mecanum_in_gazebo')
    
    # Configuration files
    hardware_controllers_config = os.path.join(mecanum_hardware_dir, 'config', 'hardware_controllers.yaml')
    lidar_config = os.path.join(mecanum_hardware_dir, 'config', 'lidar.yaml')
    imu_config = os.path.join(mecanum_hardware_dir, 'config', 'imu.yaml')
    camera_config = os.path.join(mecanum_hardware_dir, 'config', 'camera.yaml')
    
    # URDF file (from gazebo package but with hardware flag)
    urdf_file = os.path.join(mecanum_gazebo_dir, 'urdf', 'mec_rob.xacro')
    
    # Launch arguments
    use_depth_camera_arg = DeclareLaunchArgument(
        'use_depth_camera',
        default_value='false',
        description='Enable depth camera (RealSense D435/D455)'
    )
    
    esp1_port_arg = DeclareLaunchArgument(
        'esp1_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP1 (front motors)'
    )
    
    esp2_port_arg = DeclareLaunchArgument(
        'esp2_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for ESP2 (back motors)'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB2',
        description='Serial port for RPLidar A1'
    )
    
    # Robot description with hardware flag
    robot_description = Command([
        'xacro ', urdf_file,
        ' use_sim:=false',
        ' esp1_port:=', LaunchConfiguration('esp1_port'),
        ' esp2_port:=', LaunchConfiguration('esp2_port'),
        ' use_depth_camera:=', LaunchConfiguration('use_depth_camera')
    ])
    
    # ========== CORE NODES ==========
    
    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # 2. ROS2 Control Node (Hardware Interface)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            hardware_controllers_config
        ]
    )
    
    # 3. Joint State Broadcaster (t=2s)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen'
    )
    
    # 4. Mecanum Drive Controller (t=3s)
    mecanum_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='mecanum_drive_controller_spawner',
        arguments=['mecanum_drive_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen'
    )
    
    # ========== SENSOR NODES ==========
    
    # 5. RPLidar Node (t=4s)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        remappings=[('scan', '/scan')]
    )
    
    # 6. IMU Filter Node (t=4s)
    # Processes raw IMU data and publishes filtered data
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[imu_config],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data', '/imu/data')
        ]
    )
    
    # 7. IMU Publisher Node (t=4s)
    # Reads IMU data from GPIO pins on Raspberry Pi and publishes to /imu/data_raw
    # You'll need to implement this based on your IMU model (MPU6050, BNO055, etc.)
    imu_publisher = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen',
        parameters=[{'publish_rate': 100.0}]  # 100Hz IMU data
    )
    
    # 8. Depth Camera Node (t=5s) - OPTIONAL
    # Only launched if use_depth_camera:=true
    from launch.conditions import IfCondition
    
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        parameters=[camera_config],
        condition=IfCondition(LaunchConfiguration('use_depth_camera'))
    )
    
    # ========== BUILD LAUNCH DESCRIPTION ==========
    
    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(use_depth_camera_arg)
    ld.add_action(esp1_port_arg)
    ld.add_action(esp2_port_arg)
    ld.add_action(lidar_port_arg)
    
    # Core nodes (t=0s)
    ld.add_action(robot_state_publisher)
    ld.add_action(ros2_control_node)
    
    # Controllers (delayed start)
    ld.add_action(TimerAction(period=2.0, actions=[joint_state_broadcaster]))
    ld.add_action(TimerAction(period=3.0, actions=[mecanum_drive_controller]))
    
    # Sensors (delayed start)
    ld.add_action(TimerAction(period=4.0, actions=[rplidar_node]))
    ld.add_action(TimerAction(period=4.0, actions=[imu_filter_node]))
    ld.add_action(TimerAction(period=4.0, actions=[imu_publisher]))
    ld.add_action(TimerAction(period=5.0, actions=[camera_node]))
    
    return ld
