"""
Launch Cartographer for SLAM with a Mecanum (Holonomic) Robot.

This launch file starts:
1. The Cartographer SLAM node (which has internal sensor fusion).
2. The Cartographer Occupancy Grid node (to create the /map topic).
3. RViz for visualization.

This file REPLACES the EKF-based launch.
DO NOT run robot_localization (ekf_node) at the same time as this.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- 1. DEFINE FILE PATHS ---
    navigation_setup_dir = get_package_share_directory("navigation_setup")

    # Path to your Cartographer configuration (.lua file)
    cartographer_config_dir = os.path.join(navigation_setup_dir, "config")
    configuration_basename = "mecanum_cartographer_2d.lua"

    # Path to your RViz configuration
    rviz_config_file = os.path.join(navigation_setup_dir, "rviz", "slam.rviz")

    # --- 2. DECLARE LAUNCH ARGUMENTS ---
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    use_rviz = LaunchConfiguration("use_rviz", default="True")
    resolution = LaunchConfiguration("resolution", default="0.05")
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")

    return LaunchDescription(
        [
            # --- Launch Arguments ---
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="True", description="Whether to start RViz"
            ),
            # --- Cartographer Node ---
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                prefix=['gdb -ex run --args'],
                arguments=[
                    "-configuration_directory",
                    cartographer_config_dir,
                    "-configuration_basename",
                    configuration_basename,
                ],
                remappings=[
                    ("odom", "/mecanum_drive_controller/odometry"),  # <- here
                    ("imu", "/imu/data"),
                    ("scan", "/scan"),
                ],
            ),
            # --- Cartographer Occupancy Grid Node ---
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-resolution",
                    resolution,
                    "-publish_period_sec",
                    publish_period_sec,
                ],
            ),
            # --- RViz Node ---
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     arguments=["-d", rviz_config_file],
            #     parameters=[{"use_sim_time": use_sim_time}],
            #     condition=IfCondition(use_rviz),
            #     output="screen",
            # ),
        ]
    )
