"""
Remote RViz for connecting to real robot over network
"""

# not in used now later we will use this to setup the rviz on laptop to connect to robot over network
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    navigation_pkg = get_package_share_directory('navigation_setup')
    rviz_config_file = os.path.join(navigation_pkg, 'rviz', 'nav2_stvl.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    return LaunchDescription([rviz_node])