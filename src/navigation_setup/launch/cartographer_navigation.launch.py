import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # ============================================================
    # PATH CONFIGURATION
    # ============================================================
    mecanum_pkg = get_package_share_directory('mecanum_in_gazebo')
    nav_pkg = get_package_share_directory('navigation_setup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    # Config files
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    cartographer_config_dir = os.path.join(nav_pkg, 'config')
    rviz_config = os.path.join(nav_pkg, 'rviz', 'nav2.rviz')
    map_yaml = os.path.join(nav_pkg, 'maps', 'maps.yaml') # ADDED: Path to static map file
    laser_filter_config = os.path.join(nav_pkg, 'maps', 'box_filters.yaml') # Path to box filter config
    
    # Launch Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
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
    # 2. LASER FILTER (t=0s) - Added to filter self-collisions
    # ============================================================
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[
            laser_filter_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
    )
    
    # ============================================================
    # 3. CMD_VEL BRIDGE (t=5s)
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
    # 4. CARTOGRAPHER AS ODOMETRY SOURCE (t=8s)
    #    Publishes odom->base_link
    # ============================================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_odom.lua', 
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/mecanum_drive_controller/odometry'), 
            ('imu', '/imu/data')
        ]
    )
    
    # ============================================================
    # 5. NAV2 LOCALIZATION (AMCL + MAP SERVER) (t=15s)
    #    Publishes map->odom
    # ============================================================
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': map_yaml, # Supply the static map
            'autostart': 'true'
        }.items()
    )
    
    # ============================================================
    # 6. NAV2 NAVIGATION (PLANNERS + CONTROLLERS) (t=18s)
    # ============================================================
    nav2_navigation = GroupAction(
        actions=[
            # Remap cmd_vel from the controller to cmd_vel_nav
            SetRemap(src='cmd_vel', dst='cmd_vel_nav'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params,
                    'autostart': 'true',
                    'use_map_server': 'false',  # AMCL is loading the map already
                    'use_amcl': 'false',        # AMCL is launched separately
                }.items()
            )
        ]
    )

    # ============================================================
    # 7. RVIZ (t=20s)
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
    
    ld.add_action(LogInfo(msg=" TF Chain: Map(AMCL)->Odom(Carto)->Base "))
    ld.add_action(gazebo_launch)
    
    # t=5s: Bridge
    ld.add_action(TimerAction(
        period=5.0,
        actions=[cmd_vel_bridge]
    ))
    
    # t=8s: Cartographer (Odom->base_link)
    ld.add_action(TimerAction(
        period=8.0,
        actions=[cartographer_node]
    ))
    
    # t=15s: Nav2 Localization (AMCL/MapServer)
    ld.add_action(TimerAction(period=15.0, actions=[nav2_localization]))
    
    # t=18s: Nav2 Navigation (Planners/Controllers)
    ld.add_action(TimerAction(period=18.0, actions=[nav2_navigation]))
    
    # t=20s: RViz
    ld.add_action(TimerAction(period=20.0, actions=[rviz_node]))
    
    return ld