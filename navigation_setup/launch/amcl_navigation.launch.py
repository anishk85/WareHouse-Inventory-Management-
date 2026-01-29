"""
AMCL Navigation with Nav2
Simple and reliable localization using particle filter
TF chain: map (AMCL) → odom (odom_tf_pub) → base_link → sensors
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mecanum_pkg = get_package_share_directory('mecanum_in_gazebo')
    nav_pkg = get_package_share_directory('navigation_setup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(nav_pkg, 'maps', 'maps.yaml')
    rviz_config = os.path.join(nav_pkg, 'rviz', 'nav2.rviz')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Path to map yaml file'
    )
    
    # ============================================================
    # 1. GAZEBO + ROBOT (t=0s)
    # ============================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # ============================================================
    # 2. ODOM→BASE_LINK TF (t=8s) - CRITICAL!
    # ============================================================
    # odom_tf_publisher = Node(
    #     package='mecanum_in_gazebo',
    #     executable='odom_tf_publisher.py',
    #     name='odom_tf_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg, 'launch', 'ekf.launch.py')
        )
    )
    # ============================================================
    # 3. CMD_VEL BRIDGE (t=8s)
    # ============================================================
    cmd_vel_bridge = Node(
        package='mecanum_in_gazebo',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_out', '/mecanum_drive_controller/reference')
        ]
    )
    
    # ============================================================
    # 4. NAV2 LOCALIZATION (AMCL + MAP SERVER) (t=15s)
    # ============================================================
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': map_yaml,
            'autostart': 'true'
        }.items()
    )
    
    # ============================================================
    # 5. NAV2 NAVIGATION (PLANNERS + CONTROLLERS) (t=18s)
    # ============================================================
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true'
        }.items()
    )
    
    # ============================================================
    # 6. RVIZ (t=22s)
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ============================================================
    # LAUNCH SEQUENCE
    # ============================================================
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_cmd)
    
    # t=0s: Gazebo
    ld.add_action(LogInfo(msg="========================================"))
    ld.add_action(LogInfo(msg="AMCL + Nav2 Navigation"))
    ld.add_action(LogInfo(msg="TF: map (AMCL) → odom → base_link"))
    ld.add_action(LogInfo(msg="Map: " + map_file))
    ld.add_action(LogInfo(msg="========================================"))
    ld.add_action(gazebo_launch)
    
    # t=8s: TF publishers (CRITICAL - before Nav2!)
    ld.add_action(TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[8s] Starting odom→base_link TF publisher"),
            # odom_tf_publisher,
            cmd_vel_bridge
        ]
    ))
    
    ld.add_action(TimerAction(
        period=12.0,
        actions=[ekf_launch]
    ))
    
    # t=15s: AMCL localization + map server
    ld.add_action(TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="[15s] Starting AMCL localization + map server"),
            nav2_localization
        ]
    ))
    
    # t=18s: Nav2 navigation (planners, controllers, behavior trees)
    ld.add_action(TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg="[18s] Starting Nav2 navigation stack"),
            nav2_navigation
        ]
    ))
    
    # t=22s: RViz
    ld.add_action(TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg="[22s] Starting RViz"),
            LogInfo(msg="========================================"),
            LogInfo(msg="Navigation ready!"),
            LogInfo(msg="In RViz:"),
            LogInfo(msg="  1. Set initial pose: '2D Pose Estimate'"),
            LogInfo(msg="  2. Send goal: 'Nav2 Goal'"),
            LogInfo(msg="========================================"),
            rviz_node
        ]
    ))
    
    return ld
