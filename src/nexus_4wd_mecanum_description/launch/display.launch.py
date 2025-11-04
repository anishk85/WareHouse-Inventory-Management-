#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    # Package Directories
    pkg_description = get_package_share_directory('nexus_4wd_mecanum_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'nexus_4wd_mecanum.xacro')
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'nexus_display.rviz')
    controller_config_file = os.path.join(pkg_description, 'config', 'nexus_controllers.yaml')
    
    # Get the Gazebo model path
    gazebo_models_path = os.path.join(pkg_description, 'meshes')
    home_gazebo_models = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    
    # Set Gazebo model path environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_models_path + ':' + home_gazebo_models + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    # Set Gazebo resource path
    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        pkg_description + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot'
    )
    
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Z position of the robot'
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'nexus_4wd_mecanum',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    # Delay controller spawners
    load_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    load_mecanum_drive_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # Delay RViz start until controllers are loaded
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Set environment variables first
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_resource_path)
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    
    # Add nodes in order
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_entity_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_mecanum_drive_controller)
    ld.add_action(rviz_node)
    
    return ld