"""
Launch SLAM Toolbox with mecanum robot
Includes laser filters for noise removal and TF management
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_navigation_setup')
    
    # Config files
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    laser_filter_config = os.path.join(pkg_share, 'config', 'laser_filters.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to SLAM Toolbox parameters'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Laser filter node - START IMMEDIATELY
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[
            laser_filter_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', '/scan'),                      # Input: raw scan from Gazebo
            ('scan_filtered', '/scan_filtered')     # Output: filtered scan
        ]
    )
    
    # SLAM Toolbox node - uses filtered scans
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan_filtered'),  # Use filtered scan instead of raw
        ]
    )
    
    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # START LASER FILTER FIRST (immediately at t=0)
    ld.add_action(laser_filter_node)
    
    # Start SLAM after 2 seconds (wait for filtered scans to be available)
    ld.add_action(TimerAction(
        period=2.0,
        actions=[start_async_slam_toolbox_node]
    ))
    
    # Start RViz after 3 seconds
    ld.add_action(TimerAction(
        period=3.0,
        actions=[rviz_node]
    ))
    
    return ld