#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument/
from launch.substitutions import LaunchConfiguration
# from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    urdf_file_name = "mec_rob.xacro"
    # frame_prefix = LaunchConfiguration("frame_prefix", default="")

    print("urdf_file_name : {}".format(urdf_file_name))

    controller_config = os.path.join(
        get_package_share_directory("mecanum_in_gazebo"), "config", "controller.yaml"
    )

    urdf_path = os.path.join(
        get_package_share_directory("mecanum_in_gazebo"), "urdf", urdf_file_name
    )

    # with open(urdf_path, "r") as infp:
    #     robot_desc = infp.read()

    robot_desc = Command(
        ["xacro ", urdf_path, " controller_config_file:=", controller_config]
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_desc, "use_sim_time": use_sim_time}
        ],
        output="screen",
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    
    return ld
              
