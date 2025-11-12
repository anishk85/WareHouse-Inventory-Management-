"""
Complete navigation launch for real hardware
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Package directories
    mecanum_hw_pkg = get_package_share_directory('mecanum_hardware')
    navigation_pkg = get_package_share_directory('navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # File paths
    nav2_params_file = os.path.join(navigation_pkg, 'config', 'nav2_params_stvl.yaml')
    rviz_config_file = os.path.join(navigation_pkg, 'rviz', 'nav2_stvl.rviz')
    map_file = os.path.join(navigation_pkg, 'maps', 'map.yaml')
    
    # Arguments
    esp1_port = LaunchConfiguration('esp1_port')
    esp2_port = LaunchConfiguration('esp2_port')
    
    declare_esp1_port = DeclareLaunchArgument(
        'esp1_port',
        default_value='/dev/ttyUSB0',
        description='ESP1 port (FL, FR motors)'
    )
    
    declare_esp2_port = DeclareLaunchArgument(
        'esp2_port',
        default_value='/dev/ttyUSB1',
        description='ESP2 port (BL, BR motors)'
    )
    
    # 1. Launch real robot hardware interface
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_hw_pkg, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={
            'esp1_port': esp1_port,
            'esp2_port': esp2_port,
            'use_sensor_fusion': 'true'
        }.items()
    )
    
    # 2. LiDAR driver (adjust for your LiDAR model)
    # Example for RPLidar A1/A2
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB2',  # Adjust port
            'frame_id': 'laser_link',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )
    
    # 3. Depth camera driver (RealSense example)
    # Adjust for your depth camera
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'depth_module.profile': '640x480x30',
            'pointcloud.enable': True,
            'align_depth.enable': True
        }],
        remappings=[
            ('/camera/depth/color/points', '/camera_depth/points')
        ],
        output='screen'
    )
    
    # 4. IMU driver (adjust for your IMU model)
    # Example for MPU6050/MPU9250
    imu_node = Node(
        package='imu_tools',
        executable='imu_filter_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data')
        ],
        output='screen'
    )
    
    # 5. Static transform for map frame
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # 6. Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_params_file,
            'map': map_file,
            'use_respawn': 'False'
        }.items()
    )
    
    # 7. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # Launch sequence
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_esp1_port)
    ld.add_action(declare_esp2_port)
    
    # Start robot hardware immediately
    ld.add_action(real_robot_launch)
    
    # Start sensors after 3 seconds
    ld.add_action(TimerAction(
        period=3.0,
        actions=[lidar_node, camera_node, imu_node]
    ))
    
    # Start map->odom TF after 5 seconds
    ld.add_action(TimerAction(
        period=5.0,
        actions=[map_odom_tf]
    ))
    
    # Start Nav2 after 8 seconds
    ld.add_action(TimerAction(
        period=8.0,
        actions=[nav2_bringup]
    ))
    
    # Start RViz after 12 seconds
    ld.add_action(TimerAction(
        period=12.0,
        actions=[rviz_node]
    ))
    
    return ld