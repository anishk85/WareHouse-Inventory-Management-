#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    mecanum_launch_file_dir = os.path.join(
        get_package_share_directory("mecanum_in_gazebo"), "launch"
    )

    control_params_file = os.path.join(
        get_package_share_directory("mecanum_in_gazebo"), "config", "controller.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="+0.8")
    y_pose = LaunchConfiguration("y_pose", default="0")
    z_pose = LaunchConfiguration("z_pose", default="0.055")

    world = os.path.join(
        get_package_share_directory("mecanum_in_gazebo"), "arena", "arena.world"
    )

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
    #     ),
    #     launch_arguments={"world": world}.items(),
    # )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        # Pass the controllers.yaml file to Gazebo
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "--ros-args --params-file " + control_params_file,
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_launch_file_dir, "robot_state.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_launch_file_dir, "spawn.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose, "z_pose": z_pose}.items(),
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_cmd)

    return ld
