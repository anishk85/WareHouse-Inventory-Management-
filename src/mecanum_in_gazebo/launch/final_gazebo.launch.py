#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_mec = get_package_share_directory("mecanum_in_gazebo")
    
    mecanum_launch_file_dir = os.path.join(pkg_mec, "launch")
    world = os.path.join(pkg_mec, "arena", "arena.world")
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.8")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    z_pose = LaunchConfiguration("z_pose", default="0.055")
    
    # Start Gazebo server with world file
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "true"}.items(),
    )
    
    # Start Gazebo client (GUI) - delayed to let server initialize
    gzclient_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                )
            )
        ]
    )
    
    # Robot State Publisher - delayed to ensure Gazebo is ready
    robot_state_publisher_cmd = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mecanum_launch_file_dir, "robot_state.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ]
    )
    
    # Spawn robot - delayed after RSP is ready
    spawn_cmd = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mecanum_launch_file_dir, "spawn.launch.py")
                ),
                launch_arguments={
                    "x_pose": x_pose,
                    "y_pose": y_pose,
                    "z_pose": z_pose,
                }.items(),
            )
        ]
    )
    
    # Joint state broadcaster - delayed after robot spawn
    spawner_joint_state = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ]
    )
    
    # Mecanum drive controller - delayed after joint state broadcaster
    spawner_mecanum = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mecanum_drive_controller"],
                output="screen",
            )
        ]
    )
    
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_cmd,
        spawner_joint_state,
        spawner_mecanum,
    ])