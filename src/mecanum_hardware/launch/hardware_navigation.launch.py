#!/usr/bin/env python3
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
        default_value=os.path.join(navigation_setup_dir, 'maps', 'maps12.yaml'),
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
        description='Serial port for YDLIDAR'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar_link',
        description='Frame ID for LiDAR scan messages (matches URDF lidar.xacro)'
    ))
    
    # ================= 4. SENSOR NODES =================
    
    # --- CHANGED: YDLIDAR X2 NODE ---
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
            'sample_rate': 3,
            'abnormal_check_count': 4,
            'resolution_fixed': True,
            'reversion': False,
            'inverted': False,
            'isSingleChannel': True,
            'intensity': False,
            'support_motor_dtr': False,  # ✅ CRITICAL FIX: Disable DTR control (causes tremble)
            'angle_min': -180.0,
            'angle_max': 180.0,
            'range_min': 0.1,
            'range_max': 12.0,
            'frequency': 10.0,
            'connection_delay': 1000,  # ✅ CRITICAL FIX: 1 second delay for USB connection stability
            'qos_overrides./scan.publisher.reliability': 'reliable',
        }],
        remappings=[('/scan', '/scan')]
    )
    
    imu_node = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen',
        respawn=False
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
    
    # ================= 6. NAV2 BRINGUP =================
    
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
        LogInfo(msg="========================================"),
        LogInfo(msg=" NAVIGATION MODE - MECANUM ROBOT (YDLIDAR)"),
        LogInfo(msg="========================================"),
        
        # === t=0s: Start Sensors ===
        LogInfo(msg="[0s]  Starting sensors (YDLIDAR X2 + IMU)..."),
        ydlidar_node, # Replaced rplidar_node
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
                LogInfo(msg="[7s]   Starting Nav2 stack..."),
                nav2_bringup
            ]
        ),
    ])