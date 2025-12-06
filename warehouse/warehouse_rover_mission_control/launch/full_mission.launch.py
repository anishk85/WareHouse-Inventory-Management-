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
    TimerAction,
    LogInfo,
    ExecuteProcess, 
    
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    mecanum_pkg = get_package_share_directory('mecanum_gazebo')
    navigation_pkg = get_package_share_directory('mecanum_navigation_setup')
    nav_pkg = get_package_share_directory('mecanum_navigation_setup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    
    cartographer_config_dir = os.path.join(nav_pkg, 'config')
    
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    cartographer_config_dir = os.path.join(nav_pkg, 'config')
    rviz_config = os.path.join(nav_pkg, 'rviz', 'nav2.rviz')
    map_yaml = os.path.join(nav_pkg, 'maps', 'my_maps.yaml')
    
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_z = LaunchConfiguration('init_z', default='1.0')
    init_w = LaunchConfiguration('init_w', default='0.0')
    
    declare_init_z_cmd = DeclareLaunchArgument(
        'init_z', default_value='1.0', description='Initial Z orientation for AMCL'
    )
    declare_init_w_cmd = DeclareLaunchArgument(
        'init_w', default_value='0.0', description='Initial W orientation for AMCL'
    )   
    
    declare_init_x_cmd = DeclareLaunchArgument(
        'init_x', default_value='0.8', description='Initial X position for AMCL'
    )
    declare_init_y_cmd = DeclareLaunchArgument(
        'init_y', default_value='0.0', description='Initial Y position for AMCL'
    )
    
    set_initial_pose_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
            ['{header: {frame_id: "map"}, pose: {pose: {position: {x: ', init_x, ', y: ', init_y, ', z: 0.0}, orientation: {z: ', init_z, ', w: ', init_w, '}}, covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}}']
        ],
        output='screen'
    )
    
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock'
    )
    
    # ============================================================
    # 1. GAZEBO + ROBOT (t=0s)
    # ============================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_pkg, 'launch', 'simulation_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # ============================================================
    # 2. CMD_VEL BRIDGE (t=5s)
    # ============================================================
    cmd_vel_bridge = Node(
        package='mecanum_description',
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
    # 3. CARTOGRAPHER AS ODOMETRY SOURCE (t=8s)
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
    # 4. NAV2 LOCALIZATION (AMCL + MAP SERVER) (t=15s)
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
    # 5. NAV2 NAVIGATION (PLANNERS + CONTROLLERS) (t=18s)
    # ============================================================
    
    nav2_navigation = IncludeLaunchDescription(
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

    # ============================================================
    # 6. RVIZ (t=20s)
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
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
        executable='mission_controller_python.py',
        name='mission_controller_python',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_dwell_time': 3.0,
            'lift_move_time': 2.0,
        }],
        output='screen'
    )
    
    # ============================================================
    # LAUNCH SEQUENCE
    # ============================================================
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_init_x_cmd)
    ld.add_action(declare_init_y_cmd)
    ld.add_action(declare_init_z_cmd)
    ld.add_action(declare_init_w_cmd)
    
    ld.add_action(LogInfo(msg=" TF Chain: Map(AMCL)->Odom(Carto)->Base "))
    ld.add_action(gazebo_launch)
    
    # t=5s: Bridge
    
    ld.add_action(LogInfo(msg=" Gazebo launched. Starting cmd_vel bridge... "))

    
    ld.add_action(TimerAction(
        period=5.0,
        actions=[cmd_vel_bridge]
    ))
    
    ld.add_action(LogInfo(msg=" Cmd_vel bridge started. Starting Cartographer... "))
    
    # t=8s: Cartographer (Odom->base_link)
    ld.add_action(TimerAction(
        period=8.0,
        actions=[cartographer_node]
    ))
    
    ld.add_action(LogInfo(msg=" Cartographer started. Starting Nav2 Localization... "))
    
    # t=15s: Nav2 Localization (AMCL/MapServer)
    ld.add_action(TimerAction(period=15.0, actions=[nav2_localization]))
    
    ld.add_action(LogInfo(msg=" Nav2 Localization started. Starting Nav2 Navigation... "))
    
    # t=18s: Nav2 Navigation (Planners/Controllers)
    ld.add_action(TimerAction(period=18.0, actions=[nav2_navigation]))
    
    ld.add_action(LogInfo(msg=" Nav2 Navigation started. Launching RViz... "))
    
    # t=20s: RViz
    ld.add_action(TimerAction(period=20.0, actions=[rviz_node]))
    
    ld.add_action(TimerAction(
        period=22.0, 
        actions=[
            LogInfo(msg="Automatically setting initial pose..."),
            set_initial_pose_cmd
        ]
    ))
    
    # Phase 6: Start perception + database (16s)
    ld.add_action(TimerAction(
        period=25.0,
        actions=[qr_detector, inventory_db]
    ))
    
    # Phase 7: Start mission controller (20s - after everything is ready)
    ld.add_action(TimerAction(
        period=27.0,
        actions=[mission_controller]
    ))
    
    return ld