"""
Launch file to start controller manager and spawn joint_state_broadcaster in ROS2.
This is a minimal example; if you use ros2_control with a hardware interface plugin, add it via parameters.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command


def generate_launch_description():
    pkg_share = FindPackageShare(package='mecanum_in_gazebo')
    urdf = PathJoinSubstitution([pkg_share, 'urdf', 'mec_rob.xacro'])
    controller_yaml = PathJoinSubstitution([pkg_share, 'launch', 'controller.yaml'])

    # Use Command to run xacro
    robot_description = Command(['xacro ', urdf])

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # controller manager (ros2_control) - often launched via a controller_manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_yaml, {'robot_description': robot_description}],
        output='screen'
    )

    # spawn joint_state_broadcaster
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        spawner,
    ])