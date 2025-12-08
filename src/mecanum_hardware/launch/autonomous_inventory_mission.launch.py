#!/usr/bin/env python3
"""
Complete Autonomous Inventory Mission Launch File

This launches EVERYTHING needed for autonomous warehouse inventory scanning:
1. Hardware layer (motors, controllers)
2. Sensors (LiDAR, IMU, Laser distance)
3. Navigation (Nav2 + AMCL + Cartographer)
4. Actuator control (scissor lift)
5. QR detection (camera + processing)
6. Database (MongoDB Atlas)
7. Waypoint management
8. Mission orchestration

Usage:
  # First, create waypoints using joystick:
  ros2 launch mecanum_hardware create_waypoints.launch.py
  
  # Then run autonomous mission:
  ros2 launch mecanum_hardware autonomous_inventory_mission.launch.py map:=/path/to/map.yaml
"""

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
    # Package directories
    mecanum_hw_dir = get_package_share_directory('mecanum_hardware')
    navigation_setup_dir = get_package_share_directory('navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    warehouse_db_dir = get_package_share_directory('warehouse_rover_database')
    warehouse_qr_dir = get_package_share_directory('warehouse_rover_qr_detection')
    
    # Configuration files
    lidar_filter_config = os.path.join(mecanum_hw_dir, 'config', 'box_filters.yaml')
    cartographer_config_dir = os.path.join(navigation_setup_dir, 'config')
    nav2_params = os.path.join(navigation_setup_dir, 'config', 'nav2_params.yaml')
    mongodb_params = os.path.join(warehouse_db_dir, 'config', 'mongodb_params.yaml')
    
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(navigation_setup_dir, 'maps', 'warehouse_map.yaml'),
            description='Full path to map YAML file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB2',
            description='Serial port for RPLidar'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'scan_duration',
            default_value='50.0',
            description='Seconds to scan each rack'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'actuator_speed',
            default_value='50',
            description='Actuator PWM speed (0-100)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_laser_sensor',
            default_value='true',
            description='Enable laser distance sensor'
        )
    )
    
    # ================= 1. HARDWARE LAYER =================
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hw_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={'use_sim': 'false'}.items()
    )
    
    # ================= 2. SENSORS =================
    
    # RPLidar
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[('scan', '/scan_raw')]
    )
    
    # LiDAR Filter
    lidar_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[lidar_filter_config],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan')
        ]
    )
    
    # IMU
    imu_node = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen'
    )
    
    # Laser Distance Sensor
    laser_sensor_node = Node(
        package='mecanum_hardware',
        executable='laser_sensor.py',
        name='laser_distance_sensor',
        output='screen'
    )
    
    # ================= 3. ACTUATOR CONTROL =================
    actuator_node = Node(
        package='mecanum_hardware',
        executable='actuator_control_node.py',
        name='actuator_control',
        output='screen'
    )
    
    # ================= 4. NAVIGATION =================
    
    # Cartographer for odometry
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
            ('scan', '/scan'),
            ('odom', '/mecanum_drive_controller/odometry'),
            ('imu', '/imu/data')
        ]
    )
    
    # Nav2 Stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )
    
    # ================= 5. QR DETECTION =================
    qr_detection_node = Node(
        package='warehouse_rover_qr_detection',
        executable='enhanced_qr_detector.py',
        name='qr_detector',
        output='screen',
        parameters=[{
            'camera_index': 0,
            'detection_rate': 5.0,  # 5 Hz
            'min_confidence': 0.7
        }]
    )
    
    # ================= 6. DATABASE =================
    inventory_database_node = Node(
        package='warehouse_rover_database',
        executable='inventory_node_mongo.py',
        name='inventory_node',
        output='screen',
        parameters=[mongodb_params]
    )
    
    # ================= 7. WAYPOINT MANAGEMENT =================
    waypoint_manager_node = Node(
        package='mecanum_hardware',
        executable='waypoint_manager_node.py',
        name='waypoint_manager',
        output='screen',
        parameters=[{
            'waypoints_file': '/tmp/warehouse_waypoints.json',
            'auto_save': True
        }]
    )
    
    # ================= 8. MISSION MANAGER =================
    mission_manager_node = Node(
        package='mecanum_hardware',
        executable='inventory_mission_manager.py',
        name='inventory_mission_manager',
        output='screen',
        parameters=[{
            'scan_duration': LaunchConfiguration('scan_duration'),
            'actuator_speed': LaunchConfiguration('actuator_speed'),
            'lift_duration': 5.0,
            'lower_duration': 5.0
        }]
    )
    
    # ================= LAUNCH SEQUENCE =================
    
    return LaunchDescription(declared_arguments + [
        # Startup Banner
        LogInfo(msg="=" * 70),
        LogInfo(msg="    AUTONOMOUS INVENTORY MISSION SYSTEM"),
        LogInfo(msg="=" * 70),
        
        # t=0s: Hardware
        LogInfo(msg="[0s]  Starting hardware layer..."),
        hardware_launch,
        
        # t=5s: Sensors + Actuator
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[5s]  Starting sensors and actuator..."),
                rplidar_node,
                lidar_filter_node,
                imu_node,
                laser_sensor_node,
                actuator_node
            ]
        ),
        
        # t=8s: Navigation
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="[8s]  Starting navigation stack..."),
                cartographer_odom_node
            ]
        ),
        
        # t=12s: Nav2
        TimerAction(
            period=12.0,
            actions=[
                LogInfo(msg="[12s] Starting Nav2..."),
                nav2_bringup
            ]
        ),
        
        # t=15s: QR Detection + Database
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg="[15s] Starting QR detection and database..."),
                qr_detection_node,
                inventory_database_node
            ]
        ),
        
        # t=18s: Waypoint Manager
        TimerAction(
            period=18.0,
            actions=[
                LogInfo(msg="[18s] Starting waypoint manager..."),
                waypoint_manager_node
            ]
        ),
        
        # t=20s: Mission Manager
        TimerAction(
            period=20.0,
            actions=[
                LogInfo(msg="[20s] Starting mission manager..."),
                mission_manager_node
            ]
        ),
        
        # t=25s: Ready message
        TimerAction(
            period=25.0,
            actions=[
                LogInfo(msg=""),
                LogInfo(msg="=" * 70),
                LogInfo(msg="    🚀 SYSTEM READY FOR AUTONOMOUS INVENTORY MISSION"),
                LogInfo(msg="=" * 70),
                LogInfo(msg=""),
                LogInfo(msg="BEFORE STARTING MISSION:"),
                LogInfo(msg="  1. Set initial pose in RViz (2D Pose Estimate)"),
                LogInfo(msg="  2. Verify all sensors are working"),
                LogInfo(msg="  3. Check that waypoints are loaded"),
                LogInfo(msg=""),
                LogInfo(msg="START MISSION:"),
                LogInfo(msg="  ros2 service call /inventory_mission_manager/start_mission \\"),
                LogInfo(msg="    std_srvs/srv/Trigger"),
                LogInfo(msg=""),
                LogInfo(msg="STOP MISSION:"),
                LogInfo(msg="  ros2 service call /inventory_mission_manager/stop_mission \\"),
                LogInfo(msg="    std_srvs/srv/Trigger"),
                LogInfo(msg=""),
                LogInfo(msg="MONITORING:"),
                LogInfo(msg="  • Mission status: ros2 topic echo /inventory_mission_manager/status"),
                LogInfo(msg("  • QR detections: ros2 topic echo /qr_detections")),
                LogInfo(msg="  • Navigation: RViz visualization"),
                LogInfo(msg=""),
                LogInfo(msg="=" * 70)
            ]
        )
    ])
