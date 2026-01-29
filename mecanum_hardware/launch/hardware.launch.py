#!/usr/bin/env python3
"""
Launch file for Mecanum Robot with Real Hardware
Brings up:
- Robot State Publisher
- ROS2 Control (Hardware Interface)
- Mecanum Drive Controller
- CMD_VEL Bridge
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation (Gazebo) or real hardware'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='hardware_controllers.yaml',
            description='YAML file with the controllers configuration'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='mecanum_in_gazebo',
            description='Package with robot description'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='mec_rob.xacro',
            description='URDF/XACRO description file'
        )
    )
    
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'use_rviz',
    #         default_value='false',
    #         description='Start RViz2 for visualization'
    #     )
    # )
    
    # Initialize Arguments
    use_sim = LaunchConfiguration('use_sim')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    # use_rviz = LaunchConfiguration('use_rviz')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'use_sim:=',
            use_sim,
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Get controller configuration file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('mecanum_hardware'),
            'config',
            controllers_file,
        ]
    )
    
    # RViz configuration file
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(description_package), 'rviz', 'robot_view.rviz']
    # )
    
    # =========== NODES ===========
    
    # 1. Controller Manager (ROS2 Control Node)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )
    
    # 2. Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    
    # 3. Joint State Broadcaster Spawner (delayed 2 seconds after control_node starts)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # 4. Mecanum Drive Controller Spawner (delayed 3 seconds after control_node starts)
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # 5. CMD_VEL Bridge (delayed 4 seconds to ensure controller is loaded)
    cmd_vel_bridge_node = Node(
        package='mecanum_in_gazebo',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim}],
    )
    
    # 6. RViz (optional)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    #     arguments=['-d', rviz_config_file],
    #     condition=IfCondition(use_rviz),
    # )
    
    # =========== DELAYED ACTIONS ===========
    
    # Delay joint_state_broadcaster 2 seconds after control_node starts
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )
    
    # Delay mecanum_drive_controller 3 seconds after control_node starts
    delay_mecanum_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[mecanum_drive_controller_spawner],
                )
            ],
        )
    )
    
    # Delay cmd_vel_bridge 4 seconds after control_node starts
    delay_cmd_vel_bridge = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=4.0,
                    actions=[cmd_vel_bridge_node],
                )
            ],
        )
    )
    
    # =========== LAUNCH DESCRIPTION ===========
    
    nodes = [
        control_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster,
        delay_mecanum_controller,
        delay_cmd_vel_bridge,
        # rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)