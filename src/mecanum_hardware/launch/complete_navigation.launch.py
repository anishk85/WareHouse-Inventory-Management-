#!/usr/bin/env python3
"""
Complete Mapping and Navigation Launch File for Mecanum Robot
This file launches EVERYTHING needed for both mapping and navigation modes.

Usage:
  # For MAPPING mode (create a new map):
  ros2 launch mecanum_hardware complete_mapping_navigation.launch.py mode:=mapping
  
  # For NAVIGATION mode (use existing map):
  ros2 launch mecanum_hardware complete_mapping_navigation.launch.py mode:=navigation map:=/path/to/your/map.yaml

What this launches:
  1. Hardware layer (controllers, robot state publisher)
  2. Sensors (LiDAR, IMU)
  3. LiDAR filter (removes robot body from scan)
  4. Actuator control node
  5. Either Cartographer (mapping) OR Nav2+AMCL (navigation)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """Function to setup launch based on mode parameter"""
    
    # Get mode value
    mode = LaunchConfiguration('mode').perform(context)
    
    # ================= PACKAGE DIRECTORIES =================
    mecanum_hw_dir = get_package_share_directory('mecanum_hardware')
    navigation_setup_dir = get_package_share_directory('navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ================= CONFIGURATION FILES =================
    lidar_filter_config = os.path.join(mecanum_hw_dir, 'config', 'box_filters.yaml')
    cartographer_config_dir = os.path.join(navigation_setup_dir, 'config')
    nav2_params = os.path.join(navigation_setup_dir, 'config', 'nav2_params.yaml')
    
    # ================= 1. HARDWARE LAYER =================
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hw_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'use_sim': 'false',
        }.items()
    )
    
    # ================= 2. SENSOR NODES =================
    
    # YDLIDAR X2 Node (publishes to /scan_raw)
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
            'qos_overrides./scan_raw.publisher.reliability': 'reliable',
        }],
        remappings=[
            ('scan', '/scan_raw'),  # Publish raw scan data
        ]
    )
    
    # LiDAR Filter Node (filters /scan_raw -> /scan)
    lidar_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[lidar_filter_config],
        remappings=[
            ('scan', '/scan_raw'),      # Input: raw scan
            ('scan_filtered', '/scan')  # Output: filtered scan
        ]
    )
    
    # IMU Node
    imu_node = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen'
    )
    
    # ================= 3. ACTUATOR CONTROL =================
    actuator_node = Node(
        package='mecanum_hardware',
        executable='actuator_control_node.py',
        name='actuator_control',
        output='screen'
    )
    
    # ================= 4. MAPPING MODE (Cartographer) =================
    
    # Cartographer for MAPPING
    cartographer_mapping_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_mapping.lua'
        ],
        remappings=[
            ('scan', '/scan'),  # Use filtered scan
            ('odom', '/mecanum_drive_controller/odometry'),
            ('imu', '/imu/data')
        ],
        condition=IfCondition(LaunchConfiguration('is_mapping'))
    )
    
    # Occupancy Grid for MAPPING
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        condition=IfCondition(LaunchConfiguration('is_mapping'))
    )
    
    # ================= 5. NAVIGATION MODE (Cartographer Odom + Nav2) =================
    
    # Cartographer for ODOMETRY (in navigation mode)
    cartographer_odom_node = Node(
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
            ('scan', '/scan'),  # Use filtered scan
            ('odom', '/mecanum_drive_controller/odometry'),
            ('imu', '/imu/data')
        ],
        condition=IfCondition(LaunchConfiguration('is_navigation'))
    )
    
    # Nav2 Bringup (AMCL + Navigation Stack)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
        condition=IfCondition(LaunchConfiguration('is_navigation'))
    )
    
    # ================= 6. SEQUENCED STARTUP =================
    
    nodes_to_launch = []
    
    # Startup messages based on mode
    if mode == 'mapping':
        nodes_to_launch.extend([
            LogInfo(msg="========================================"),
            LogInfo(msg="    MAPPING MODE - MECANUM ROBOT"),
            LogInfo(msg="========================================"),
            LogInfo(msg="Starting components in sequence:"),
            LogInfo(msg="  [0s]  Hardware layer"),
            LogInfo(msg="  [5s]  Sensors + Actuator + LiDAR Filter"),
            LogInfo(msg="  [8s]  Cartographer SLAM"),
            LogInfo(msg="========================================"),
        ])
    else:  # navigation
        nodes_to_launch.extend([
            LogInfo(msg="========================================"),
            LogInfo(msg="  NAVIGATION MODE - MECANUM ROBOT"),
            LogInfo(msg="========================================"),
            LogInfo(msg="Starting components in sequence:"),
            LogInfo(msg="  [0s]  Hardware layer"),
            LogInfo(msg="  [5s]  Sensors + Actuator + LiDAR Filter"),
            LogInfo(msg="  [8s]  Cartographer Odometry"),
            LogInfo(msg="  [12s] Nav2 Stack (AMCL + Navigation)"),
            LogInfo(msg="========================================"),
        ])
    
    # t=0s: Hardware
    nodes_to_launch.append(hardware_launch)
    
    # t=5s: Sensors + Actuator + Filter
    nodes_to_launch.append(
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[5s]  Starting sensors, actuator, and LiDAR filter..."),
                ydlidar_node,
                lidar_filter_node,
                imu_node,
                actuator_node
            ]
        )
    )
    
    # t=8s: SLAM or Odometry
    if mode == 'mapping':
        nodes_to_launch.append(
            TimerAction(
                period=8.0,
                actions=[
                    LogInfo(msg="[8s]  Starting Cartographer SLAM for mapping..."),
                    cartographer_mapping_node,
                    occupancy_grid_node
                ]
            )
        )
        
        # t=15s: Ready message for mapping
        nodes_to_launch.append(
            TimerAction(
                period=15.0,
                actions=[
                    LogInfo(msg=""),
                    LogInfo(msg="========================================"),
                    LogInfo(msg="  MAPPING SYSTEM READY!"),
                    LogInfo(msg="========================================"),
                    LogInfo(msg=""),
                    LogInfo(msg="INSTRUCTIONS:"),
                    LogInfo(msg="  1. Drive robot around using teleop:"),
                    LogInfo(msg="     ros2 run teleop_twist_keyboard teleop_twist_keyboard"),
                    LogInfo(msg=""),
                    LogInfo(msg="  2. Control scissor lift actuator:"),
                    LogInfo(msg="     ros2 topic pub /actuator/command std_msgs/String \"data: 'up'\""),
                    LogInfo(msg="     ros2 topic pub /actuator/command std_msgs/String \"data: 'down'\""),
                    LogInfo(msg="     ros2 topic pub /actuator/command std_msgs/String \"data: 'stop'\""),
                    LogInfo(msg=""),
                    LogInfo(msg="  3. When done, save the map:"),
                    LogInfo(msg="     ros2 run nav2_map_server map_saver_cli -f ~/my_map"),
                    LogInfo(msg=""),
                    LogInfo(msg="MONITORING:"),
                    LogInfo(msg="  • View in RViz: ros2 launch nav2_bringup rviz_launch.py"),
                    LogInfo(msg("  • Topics: /scan (filtered), /map, /tf")),
                    LogInfo(msg="========================================")
                ]
            )
        )
    else:  # navigation
        nodes_to_launch.append(
            TimerAction(
                period=8.0,
                actions=[
                    LogInfo(msg="[8s]  Starting Cartographer odometry fusion..."),
                    cartographer_odom_node
                ]
            )
        )
        
        # t=12s: Nav2
        nodes_to_launch.append(
            TimerAction(
                period=12.0,
                actions=[
                    LogInfo(msg="[12s] Starting Nav2 navigation stack..."),
                    nav2_bringup
                ]
            )
        )
        
        # t=20s: Ready message for navigation
        nodes_to_launch.append(
            TimerAction(
                period=20.0,
                actions=[
                    LogInfo(msg=""),
                    LogInfo(msg="========================================"),
                    LogInfo(msg="  NAVIGATION SYSTEM READY!"),
                    LogInfo(msg="========================================"),
                    LogInfo(msg=""),
                    LogInfo(msg="STEP 1: SET INITIAL POSE IN RVIZ"),
                    LogInfo(msg="  • Click '2D Pose Estimate' button"),
                    LogInfo(msg="  • Click on map where robot is"),
                    LogInfo(msg="  • Drag to set orientation"),
                    LogInfo(msg=""),
                    LogInfo(msg="STEP 2: SEND NAVIGATION GOAL"),
                    LogInfo(msg="  • Click 'Nav2 Goal' button"),
                    LogInfo(msg="  • Click destination on map"),
                    LogInfo(msg="  • Robot will navigate autonomously!"),
                    LogInfo(msg=""),
                    LogInfo(msg="ACTUATOR CONTROL:"),
                    LogInfo(msg="  ros2 topic pub /actuator/command std_msgs/String \"data: 'up'\""),
                    LogInfo(msg="  ros2 topic pub /actuator/command std_msgs/String \"data: 'down'\""),
                    LogInfo(msg="  ros2 topic pub /actuator/command std_msgs/String \"data: 'stop'\""),
                    LogInfo(msg("========================================"))
                ]
            )
        )
    
    return nodes_to_launch


def generate_launch_description():
    # ================= LAUNCH ARGUMENTS =================
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            description='Mode: "mapping" or "navigation"'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('navigation_setup'),
                'maps',
                'maps.yaml'
            ),
            description='Full path to map YAML file (only used in navigation mode)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB2',
            description='Serial port for YDLIDAR'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='lidar_link',
            description='Frame ID for LiDAR scan messages (matches URDF lidar.xacro)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start Nav2 lifecycle nodes'
        )
    )
    
    # Helper arguments for conditional logic
    declared_arguments.append(
        DeclareLaunchArgument(
            'is_mapping',
            default_value='true',  # Will be overridden by OpaqueFunction
            description='Internal flag for mapping mode'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'is_navigation',
            default_value='false',  # Will be overridden by OpaqueFunction
            description='Internal flag for navigation mode'
        )
    )
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])