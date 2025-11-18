"""
Launch SLAM Toolbox with EKF for odometry fusion.

This launch file starts:
1. The robot_localization EKF node to fuse wheel odometry and IMU.
2. The SLAM Toolbox node for mapping.
3. RViz for visualization.

This is the standard, robust setup for mapping.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    navigation_setup_dir = get_package_share_directory('navigation_setup')
    
    # --- 1. DEFINE FILE PATHS ---
    
    # Path to your SLAM parameters
    slam_params_file = os.path.join(navigation_setup_dir, 'config', 'mapper_params_online_async.yaml')
    
    # Path to your RViz configuration
    rviz_config_file = os.path.join(navigation_setup_dir, 'rviz', 'slam.rviz')
    
    # Path to your EKF (robot_localization) parameters
    ekf_params_file = os.path.join(navigation_setup_dir, 'config', 'ekf.yaml')

    # --- 2. DECLARE LAUNCH ARGUMENTS ---

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz'
    )
    
    # --- 3. DEFINE NODES ---

    # Start the EKF (robot_localization) node
    # This node fuses raw odometry and IMU data to produce a
    # clean, filtered odometry estimate and the odom -> base_link TF.
    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        
        # This remapping is CRUCIAL.
        # It takes the EKF's "fixed" odometry (/odom/filtered)
        # and publishes it on the /odom topic.
        # SLAM Toolbox gets its odometry data from the /odom -> /base_link
        # TF published by this node.
        
        remappings=[('/odom/filtered', '/odom')]
    )

    # Start the SLAM Toolbox node

    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )
    
    # Start RViz node
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # --- 4. CREATE LAUNCH DESCRIPTION ---
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    
    # Add nodes to the launch description
    ld.add_action(start_ekf_node)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(rviz_node)
    
    return ld