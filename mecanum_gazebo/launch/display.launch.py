"""
Simple ROS 2 launch to display the robot model with rviz and start robot_state_publisher
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('mecanum_gazebo')
    

    urdf_file = os.path.join(pkg_share, 'urdf', 'mec_rob.xacro')
    rviz_config = os.path.join(pkg_share, 'launch', 'urdf.rviz')
  
    controller_config = os.path.join(pkg_share, 'config', 'controllers.yaml')
    robot_description = Command([
        'xacro ', urdf_file,
        ' controller_config_file:=', controller_config
    ])

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description}]
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz_node,
    ])