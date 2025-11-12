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
    mecanum_pkg = get_package_share_directory('mecanum_in_gazebo')
    navigation_pkg = get_package_share_directory('navigation_setup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    

    nav2_params_file = os.path.join(navigation_pkg, 'config', 'nav2_params_stvl.yaml')
    rviz_config_file = os.path.join(navigation_pkg, 'rviz', 'nav2_stvl.rviz')  # Changed
    map_file = os.path.join(navigation_pkg, 'maps', 'map.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock'
    )
    
    # 1. Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg, 'launch', 'gazebo.launch.py')
        )
    )
    
    # 2. Odom TF Publisher
    odom_tf_publisher = Node(
        package='mecanum_in_gazebo',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 3. CMD_VEL to Mecanum converter with INCREASED SPEED
    cmd_vel_converter = Node(
        package='mecanum_in_gazebo',
        executable='cmd_vel_to_mecanum.py',
        name='cmd_vel_to_mecanum',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_separation_x': 0.3,
            'wheel_separation_y': 0.25,
            'wheel_radius': 0.05
        }],
        output='screen'
    )
    
    # 4. Static transform for map frame (temporary, AMCL will replace this)
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 5. Nav2 Bringup with STVL config
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
    
    # 6. RViz with STVL visualization
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
    
    # Start Gazebo immediately
    ld.add_action(gazebo_launch)
    
    # Start odom TF + cmd_vel converter after 8 seconds
    ld.add_action(TimerAction(
        period=8.0,
        actions=[odom_tf_publisher, cmd_vel_converter]
    ))
    
    # Start map->odom TF after 9 seconds
    ld.add_action(TimerAction(
        period=9.0,
        actions=[map_odom_tf]
    ))
    
    # Start Nav2 after 12 seconds
    ld.add_action(TimerAction(
        period=12.0,
        actions=[nav2_bringup]
    ))
    
    # Start RViz after 15 seconds
    ld.add_action(TimerAction(
        period=15.0,
        actions=[rviz_node]
    ))
    
    return ld