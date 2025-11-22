"""
MASTER LAUNCH: Complete Autonomous Warehouse Scanning System
Combines: Gazebo → Navigation → QR Detection → Database → Mission Control
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
    
    # File paths
    nav2_params_file = os.path.join(navigation_pkg, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(navigation_pkg, 'rviz', 'nav2.rviz')
    map_file = os.path.join(navigation_pkg, 'maps', 'map.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock'
    )
    
    # ════════════════════════════════════════════════════════════════
    # PHASE 1: SIMULATION & ROBOT (0s)
    # ════════════════════════════════════════════════════════════════
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg, 'launch', 'gazebo.launch.py')
        )
    )
    
    # ════════════════════════════════════════════════════════════════
    # PHASE 2: ODOMETRY & MOTOR CONTROL (8s)
    # ════════════════════════════════════════════════════════════════
    
    odom_tf_publisher = Node(
        package='mecanum_in_gazebo',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
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
    
    # ════════════════════════════════════════════════════════════════
    # PHASE 3: LOCALIZATION (9s)
    # ════════════════════════════════════════════════════════════════
    
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ════════════════════════════════════════════════════════════════
    # PHASE 4: NAVIGATION STACK (12s)
    # ════════════════════════════════════════════════════════════════
    
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
    
    # ════════════════════════════════════════════════════════════════
    # PHASE 5: VISUALIZATION (15s)
    # ════════════════════════════════════════════════════════════════
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ════════════════════════════════════════════════════════════════
    # PHASE 6: PERCEPTION & DATABASE (16s)
    # ════════════════════════════════════════════════════════════════
    
    # QR Detection on lift camera
    qr_detector = Node(
        package='warehouse_rover_image_processing',
        executable='qr_detector_enhanced_node',
        name='qr_detector_lift',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_visualization': True,
            'enable_ipt': True,
            'enable_multipass': True,
            'min_qr_size': 50,
            'max_qr_size': 500
        }],
        remappings=[
            ('/camera/image_raw', '/lift_camera/image_raw'),
        ],
        output='screen'
    )
    
    # Inventory Database
    inventory_db = Node(
        package='warehouse_rover_database',
        executable='inventory_node',
        name='inventory',
        parameters=[{
            'use_sim_time': use_sim_time,
            'database_path': '/tmp/warehouse_inventory.db',
            'auto_export': True,
            'export_dir': '/tmp/inventory_exports',
            'mission_name': 'AUTONOMOUS_SCAN'
        }],
        output='screen'
    )
    
    # ════════════════════════════════════════════════════════════════
    # PHASE 7: MISSION CONTROLLER (20s - Wait for Nav2 to be ready)
    # ════════════════════════════════════════════════════════════════
    
    mission_controller = Node(
        package='warehouse_rover_mission_control',
        executable='mission_controller_node',
        name='mission_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_dwell_time': 3.0,
            'lift_move_time': 2.0,
        }],
        output='screen'
    )
    
    # ════════════════════════════════════════════════════════════════
    # BUILD LAUNCH DESCRIPTION WITH TIMED STARTUP
    # ════════════════════════════════════════════════════════════════
    
    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Phase 1: Start Gazebo immediately
    ld.add_action(gazebo_launch)
    
    # Phase 2: Start odometry + motor control (8s)
    ld.add_action(TimerAction(
        period=8.0,
        actions=[odom_tf_publisher, cmd_vel_converter]
    ))
    
    # Phase 3: Start localization TF (9s)
    ld.add_action(TimerAction(
        period=9.0,
        actions=[map_odom_tf]
    ))
    
    # Phase 4: Start Nav2 stack (12s)
    ld.add_action(TimerAction(
        period=12.0,
        actions=[nav2_bringup]
    ))
    
    # Phase 5: Start RViz (15s)
    ld.add_action(TimerAction(
        period=15.0,
        actions=[rviz_node]
    ))
    
    # Phase 6: Start perception + database (16s)
    ld.add_action(TimerAction(
        period=16.0,
        actions=[qr_detector, inventory_db]
    ))
    
    # Phase 7: Start mission controller (20s - after everything is ready)
    ld.add_action(TimerAction(
        period=20.0,
        actions=[mission_controller]
    ))
    
    return ld
