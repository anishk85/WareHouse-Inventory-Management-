#!/usr/bin/env python3

# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import (
    TimerAction,
    RegisterEventHandler,
)
import os
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    # pkg_share = get_package_share_directory("mecanum_gazebo")
    pkg_mecanum = get_package_share_directory("mecanum_gazebo")
    control_pkg = get_package_share_directory("mecanum_description")

    # Define file paths

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw_pose = LaunchConfiguration('yaw_pose', default='3.14159')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_x_position_cmd = DeclareLaunchArgument(
        "x_pose", default_value="+0.8", description="X position of the robot"
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        "y_pose", default_value="0.0", description="Y position of the robot"
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        "z_pose", default_value="0.055", description="Z position of the robot"
    )

    # 2. Spawn Entity (the robot)
    # This node reads the /robot_description topic and spawns the entity
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "mec_rob",
            "-topic",
            "robot_description",  # Spawn from topic, not file
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
            "-Y", # rotated to face forward
            yaw_pose,  # Face forward
            "-timeout",
            "60.0",
        ],
        output="screen",
    )

    # --- Launch Description ---
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    # Add nodes
    # ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    
    load_joint_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    control_params_file = os.path.join(control_pkg, "config", "controllers.yaml")

    load_mecanum_controller_cmd =Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager", "/controller_manager",
            "--ros-args",
            "--params-file", control_params_file
        ],
        output="screen",
    )
    
    load_lift_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    twist_mux_pkg = 'mecanum_description'

    # 2. Find the full path to the configuration file
    twist_mux_config = os.path.join(
        get_package_share_directory(twist_mux_pkg),
        'config',
        'twist_mux.yaml'
    )
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        
    )
    
    ld.add_action(TimerAction(period=5.0, actions=[load_joint_broadcaster_cmd]))
    
    
    ld.add_action(TimerAction(period=8.0, actions=[load_mecanum_controller_cmd]))
    
    ld.add_action(TimerAction(period=10.0, actions=[load_lift_controller_cmd]))
    
    
    # ld.add_action(TimerAction(period=12.0, actions=[twist_mux_node]))
    
     # CMD_VEL Bridge - converts /cmd_vel to /mecanum_drive_controller/reference
    cmd_vel_bridge_node = Node(
        package='mecanum_description',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # SENSOR Bridge - publishes mecanum odometry to /odom and fixes IMU covariances
    sensor_bridge_node = Node(
        package='mecanum_description',
        executable='sensor_bridge.py',
        name='sensor_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    
    ld.add_action(TimerAction(period=14.0, actions=[cmd_vel_bridge_node]))
    
    
    # Start sensor bridge to publish odometry to /odom
    ld.add_action(TimerAction(period=16.0, actions=[sensor_bridge_node]))
    

    return ld
