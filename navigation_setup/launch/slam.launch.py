"""
Launch SLAM Toolbox with mecanum robot
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
    

    slam_params_file = os.path.join(navigation_setup_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(navigation_setup_dir, 'rviz', 'slam.rviz')
    

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
    
  
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # Add nodes
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(rviz_node)
    
    return ld