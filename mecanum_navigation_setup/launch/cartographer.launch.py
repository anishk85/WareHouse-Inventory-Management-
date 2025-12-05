#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package directories
    mecanum_gazebo_dir = get_package_share_directory('mecanum_gazebo')
    mecanum_navigation_setup_dir = get_package_share_directory('mecanum_navigation_setup')
    
    # Configuration files
    cartographer_config_dir = os.path.join(mecanum_navigation_setup_dir, 'config')
    cartographer_config_file = 'cartographer_mapping.lua'  # New config file name
    rviz_config_file = os.path.join(mecanum_navigation_setup_dir, 'rviz', 'cartographer.rviz')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    # 1. Launch Gazebo simulation (includes sensor_bridge)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(mecanum_gazebo_dir, 'launch', 'simulation_world.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    # )
    
    # 2. Cartographer Node (SLAM + sensor fusion + odom TF publishing)
    # NOW publishes both map→odom AND odom→base_link transforms
    # Launches at t=15s to allow Gazebo and controllers to stabilize
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_file
        ],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'mecanum_drive_controller/odometry'),  # Read controller odometry
            ('imu', 'imu/data')  # From Gazebo IMU plugin
        ]
    )
    
    cartographer_node_delayed = TimerAction(
        period=15.0,
        actions=[cartographer_node]
    )
    
    # 3. Cartographer Occupancy Grid Node
    # Creates occupancy grid from Cartographer submaps for navigation
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-resolution', '0.05']
    )
    
    
    
    occupancy_grid_delayed = TimerAction(
        period=18.0,
        actions=[occupancy_grid_node]
    )
    
    
    # 4. RViz for visualization
    # Launches at t=22s after Cartographer is running
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    rviz_delayed = TimerAction(
        period=22.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        # gazebo_launch,              # t=0s:  Gazebo + robot + controllers + sensor_bridge
        cartographer_node_delayed,  # t=15s: Cartographer SLAM (now publishes odom→base_link too!)
        occupancy_grid_delayed,     # t=18s: Occupancy grid generation
        rviz_delayed                # t=22s: Visualization
        # REMOVED: odom_tf_publisher (no longer needed!) we will check this in hardware about it performance 
    ])