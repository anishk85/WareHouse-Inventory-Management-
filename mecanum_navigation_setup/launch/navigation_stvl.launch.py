"""
Complete navigation launch with STVL (depth camera support) - starts Gazebo, robot, and Nav2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    mecanum_pkg = get_package_share_directory('mecanum_gazebo')
    navigation_pkg = get_package_share_directory('mecanum_navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    

    nav2_params_file = os.path.join(navigation_pkg, 'config', 'nav2_params_stvl.yaml')
    rviz_config_file = os.path.join(navigation_pkg, 'rviz', 'nav2_stvl.rviz')  # Changed
    map_file = os.path.join(navigation_pkg, 'maps', 'maps.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock'
    )
    
    # 1. Launch Gazebo with robot (includes controllers, cmd_vel_bridge, sensor_bridge)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg, 'launch', 'gazebo.launch.py')
        )
    )
    
    # 2. Launch EKF for sensor fusion (publishes odom->base_link TF)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg, 'launch', 'ekf.launch.py')
        )
    )
    
    # 3. Nav2 Bringup with STVL config (includes depth camera obstacle layer)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'true',
            'params_file': nav2_params_file,
            'map': map_file,
            'use_respawn': 'False'
        }.items()
    )
    
    # 4. RViz with STVL visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create launch description with timed actions
    ld = LaunchDescription()
    
    # Add declaration
    ld.add_action(declare_use_sim_time_cmd)
    
    # Start Gazebo immediately (0 seconds)
    # This includes: robot, controllers, cmd_vel_bridge, sensor_bridge
    ld.add_action(gazebo_launch)
    
    # Start EKF after 12 seconds (wait for controllers to be active)
    # EKF will publish odom->base_link TF
    ld.add_action(TimerAction(
        period=12.0,
        actions=[ekf_launch]
    ))
    
    # Start Nav2 after 18 seconds (wait for EKF TF to be stable)
    # AMCL will publish map->odom TF
    ld.add_action(TimerAction(
        period=18.0,
        actions=[nav2_bringup]
    ))
    
    # Start RViz after 24 seconds (wait for Nav2 to initialize)
    ld.add_action(TimerAction(
        period=24.0,
        actions=[rviz_node]
    ))
    
    return ld