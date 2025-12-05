"""
Launch file to start controller manager and spawn controllers in ROS2.
Use this for standalone testing without Gazebo.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_gazebo')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mec_rob.xacro')
    controller_config = os.path.join(pkg_share, 'launch', 'controllers.yaml')


    robot_description = Command([
        'xacro ', urdf_file,
        ' controller_config_file:=', controller_config
    ])

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
        output='screen'
    )

    # controller manager (ros2_control)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config
        ],
        output='screen'
    )

    # spawn joint_state_broadcaster with delay
    spawn_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        spawn_joint_state_broadcaster,
    ])