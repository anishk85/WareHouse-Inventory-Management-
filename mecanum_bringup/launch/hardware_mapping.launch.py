#!/usr/bin/env python3
"""
Hardware Mapping Launch - Cartographer SLAM on real robot
Uses IMU + LiDAR + wheel odometry for robust mapping

TF chain: map -> odom -> base_link (all published by Cartographer)
- map->odom: SLAM correction
- odom->base_link: Sensor fusion (IMU + wheels + LiDAR)

Wireless RViz: Run RViz on laptop with same ROS_DOMAIN_ID
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    mecanum_hardware_dir = get_package_share_directory('mecanum_hardware')
    mecanum_navigation_setup_dir = get_package_share_directory('mecanum_navigation_setup')
    
    # Configuration files
    cartographer_config_dir = os.path.join(mecanum_navigation_setup_dir, 'config')
    cartographer_config_file = 'cartographer_mapping.lua'  # Full SLAM with map->odom->base_link
    rviz_config_file = os.path.join(mecanum_navigation_setup_dir, 'rviz', 'cartographer.rviz')
    
    # ========== LAUNCH ARGUMENTS ==========
    use_depth_camera_arg = DeclareLaunchArgument(
        'use_depth_camera',
        default_value='false',
        description='Enable depth camera for mapping'
    )
    
    esp1_port_arg = DeclareLaunchArgument(
        'esp1_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP1 (FL+FR motors)'
    )
    
    esp2_port_arg = DeclareLaunchArgument(
        'esp2_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for ESP2 (BL+BR motors)'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB2',
        description='Serial port for RPLidar'
    )
    
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='42',
        description='ROS_DOMAIN_ID for wireless communication (0-101, default: 42)'
    )
    
    launch_remote_rviz_arg = DeclareLaunchArgument(
        'launch_remote_rviz',
        default_value='false',
        description='Launch RViz on laptop via SSH (requires ssh key setup)'
    )
    
    laptop_user_arg = DeclareLaunchArgument(
        'laptop_user',
        default_value='user',
        description='Username on laptop for SSH RViz launch'
    )
    
    laptop_ip_arg = DeclareLaunchArgument(
        'laptop_ip',
        default_value='192.168.1.100',
        description='Laptop IP address for SSH RViz launch'
    )
    
    # ========== 1. HARDWARE + SENSORS (t=0s) ==========
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hardware_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'use_depth_camera': LaunchConfiguration('use_depth_camera'),
            'esp1_port': LaunchConfiguration('esp1_port'),
            'esp2_port': LaunchConfiguration('esp2_port'),
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items()
    )
    
    # ========== 2. CARTOGRAPHER SLAM (t=8s) ==========
    # Performs sensor fusion and publishes complete TF tree
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
            ('scan', '/scan'),                              # From RPLidar
            ('odom', '/mecanum_drive_controller/odom'),     # Pure wheel odometry
            ('imu', '/imu/data')                            # Raw IMU data
        ]
    )
    
    cartographer_delayed = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[8s]  Starting Cartographer SLAM with sensor fusion..."),
            cartographer_node
        ]
    )
    
    # ========== 3. CARTOGRAPHER OCCUPANCY GRID (t=10s) ==========
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    occupancy_grid_delayed = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[10s] 🗺️  Starting occupancy grid generation..."),
            occupancy_grid_node
        ]
    )
    
    # ========== 4. REMOTE RVIZ LAUNCHER (OPTIONAL, t=12s) ==========
    # SSH into laptop and launch RViz with correct ROS_DOMAIN_ID
    # remote_rviz_cmd = ExecuteProcess(
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('launch_remote_rviz')),
    #     cmd=[
    #         'ssh',
    #         [LaunchConfiguration('laptop_user'), '@', LaunchConfiguration('laptop_ip')],
    #         f'source /opt/ros/humble/setup.bash && '
    #         f'export ROS_DOMAIN_ID={LaunchConfiguration("ros_domain_id")} && '
    #         f'rviz2 -d {rviz_config_file}'
    #     ],
    #     output='screen'
    # )
    
    # remote_rviz_delayed = TimerAction(
    #     period=12.0,
    #     actions=[
    #         LogInfo(msg="[12s] 🖥️  Attempting to launch RViz on laptop..."),
    #         remote_rviz_cmd
    #     ]
    # )
    
      
    completion_info = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=""),
            LogInfo(msg=" All systems operational!"),
            LogInfo(msg=" Drive the robot to build the map."),
            LogInfo(msg="")
        ]
    )
    
    # ========== BUILD LAUNCH DESCRIPTION ==========
    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(use_depth_camera_arg)
    ld.add_action(esp1_port_arg)
    ld.add_action(esp2_port_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(ros_domain_id_arg)
    ld.add_action(launch_remote_rviz_arg)
    ld.add_action(laptop_user_arg)
    ld.add_action(laptop_ip_arg)
    
    # Startup info
   
    
    # Launch sequence
    ld.add_action(hardware_launch)           # t=0s: Hardware + sensors
    ld.add_action(cartographer_delayed)      # t=8s: SLAM
    ld.add_action(occupancy_grid_delayed)    # t=10s: Occupancy grid
    # ld.add_action(remote_rviz_delayed)       # t=12s: Remote RViz (optional)
    ld.add_action(completion_info)           # t=12s: Completion message
    
    return ld