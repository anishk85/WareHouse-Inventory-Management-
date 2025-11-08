"""
Simple ROS 2 launch to display the robot model with rviz and start robot_state_publisher
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='mecanum_in_gazebo')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mec_rob.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'launch', 'urdf.rviz'])
    

    # Use Command to run xacro and get the robot description
    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

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