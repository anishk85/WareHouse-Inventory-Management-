#!/usr/bin/env python3
"""
Complete Warehouse Robot Launch File
Combines: Hardware Control + Mapping (SLAM) + Waypoint Recording

This launch file brings up the entire system:
1. Hardware Interface (Robot State Publisher, ROS2 Control, Mecanum Controller)
2. Sensors (RPLidar, IMU)
3. SLAM (Cartographer)
4. Waypoint Recording System
5. Optional: Teleop with waypoint trigger

Usage:
    # Full system with teleop
    ros2 launch your_package complete_mapping_system.launch.py

    # Without teleop (use joystick or other controller)
    ros2 launch your_package complete_mapping_system.launch.py use_teleop:=false
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    RegisterEventHandler, 
    TimerAction,
    LogInfo,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (
    Command, 
    FindExecutable, 
    LaunchConfiguration, 
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ==================== ARGUMENTS ====================
    declared_arguments = []
    
    # General
    declared_arguments.append(DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Use simulation (Gazebo) or real hardware'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'use_teleop', default_value='true',
        description='Launch keyboard teleop with waypoint recording'))
    
    # Hardware/Description
    declared_arguments.append(DeclareLaunchArgument(
        'controllers_file', default_value='hardware_controllers.yaml',
        description='YAML file with the controllers configuration'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'description_package', default_value='mecanum_in_gazebo',
        description='Package with robot description'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'description_file', default_value='mec_rob.xacro',
        description='URDF/XACRO description file'))
    
    # Sensor Configuration
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB2',
        description='Serial port for RPLidar'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_frame', default_value='lidar_link',
        description='Frame ID for LiDAR scan messages'))
    
    # Waypoint Configuration
    declared_arguments.append(DeclareLaunchArgument(
        'waypoint_dir', default_value=os.path.expanduser('~/.ros/waypoints'),
        description='Directory to store waypoint files'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'use_odom_for_waypoints', default_value='true',
        description='Use odometry instead of TF for waypoint recording'))
    
    # Get configurations
    use_sim = LaunchConfiguration('use_sim')
    use_teleop = LaunchConfiguration('use_teleop')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_frame = LaunchConfiguration('lidar_frame')
    waypoint_dir = LaunchConfiguration('waypoint_dir')
    use_odom_for_waypoints = LaunchConfiguration('use_odom_for_waypoints')
    
    # ==================== PATHS ====================
    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare(description_package), 
            'urdf', 
            description_file
        ]),
        ' use_sim:=', use_sim,
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Controller configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('mecanum_hardware'),
        'config',
        controllers_file,
    ])
    
    # Cartographer configuration
    pkg_nav = 'navigation_setup'
    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare(pkg_nav), 
        'config'
    ])
    cartographer_config_file = 'cartographer_mapping.lua'
    
    # ==================== HARDWARE NODES ====================
    
    # 1. Controller Manager (ROS2 Control Node)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        respawn=False,
    )
    
    # 2. Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    
    # 3. Joint State Broadcaster Spawner (delayed)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # 4. Mecanum Drive Controller Spawner (delayed)
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # 5. CMD_VEL Bridge (delayed)
    cmd_vel_bridge_node = Node(
        package='mecanum_in_gazebo',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim}],
    )
    
    # ==================== SENSOR NODES ====================
    
    # 6. RPLidar Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': lidar_frame,
            'inverted': False,
            'angle_compensate': True,
        }]
    )
    
    # 7. IMU Node
    imu_node = Node(
        package='mecanum_hardware',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen'
    )
    
    # ==================== SLAM NODES ====================
    
    # 8. Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_file
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/mecanum_drive_controller/odometry'),
            ('imu', '/imu/data')
        ]
    )
    
    # 9. Occupancy Grid Node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    # ==================== WAYPOINT RECORDING ====================
    
    # 10. Waypoint Recorder Node
    waypoint_recorder_node = Node(
        package='mecanum_hardware',
        executable='waypoint_recorder_node.py',
        name='waypoint_recorder',
        output='screen',
        parameters=[{
            'waypoint_dir': waypoint_dir,
            'use_odom': use_odom_for_waypoints,
            'odom_topic': '/mecanum_drive_controller/odometry',
            'base_frame': 'base_link',
            'map_frame': 'map'
        }]
    )
    
    # # 11. Teleop with Waypoint Recording (Optional)
    # teleop_node = Node(
    #     package='mecanum_in_gazebo',
    #     executable='enhanced_joystick_teleop.py',
    #     name='teleop_joy',
    #     output='screen',
    #     parameters=[{
    #         'linear_speed': 0.3,
    #         'angular_speed': 0.5
    #     }],
    #     condition=IfCondition(use_teleop)
    # )
    
    # ==================== DELAYED ACTIONS ====================
    
    # Delay joint_state_broadcaster 2 seconds after control_node starts
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        LogInfo(msg='---> Starting Joint State Broadcaster'),
                        joint_state_broadcaster_spawner
                    ],
                )
            ],
        )
    )
    
    # Delay mecanum_drive_controller 3 seconds after control_node starts
    delay_mecanum_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        LogInfo(msg='---> Starting Mecanum Drive Controller'),
                        mecanum_drive_controller_spawner
                    ],
                )
            ],
        )
    )
    
    # Delay cmd_vel_bridge 4 seconds after control_node starts
    delay_cmd_vel_bridge = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=4.0,
                    actions=[
                        LogInfo(msg='---> Starting CMD_VEL Bridge'),
                        cmd_vel_bridge_node
                    ],
                )
            ],
        )
    )
    
    # Delay sensors 5 seconds after hardware is ready
    delay_sensors = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg='---> Starting Sensors (LiDAR + IMU)'),
                        rplidar_node,
                        imu_node
                    ],
                )
            ],
        )
    )
    
    # Delay SLAM 8 seconds after control_node (sensors need time to stabilize)
    delay_slam = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[
                        LogInfo(msg='---> Starting SLAM (Cartographer)'),
                        cartographer_node,
                        occupancy_grid_node
                    ],
                )
            ],
        )
    )
    
    # Delay waypoint recorder 10 seconds (after SLAM is running)
    delay_waypoint_recorder = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=10.0,
                    actions=[
                        LogInfo(msg='---> Starting Waypoint Recorder'),
                        waypoint_recorder_node
                    ],
                )
            ],
        )
    )
    
    # Delay teleop 11 seconds (last, after everything is ready)
    # delay_teleop = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=control_node,
    #         on_start=[
    #             TimerAction(
    #                 period=11.0,
    #                 actions=[
    #                     LogInfo(msg='---> Starting Teleop (if enabled)'),
    #                     teleop_node
    #                 ],
    #             )
    #         ],
    #     )
    # )
    
    # ==================== LAUNCH DESCRIPTION ====================
    
    nodes = [
        LogInfo(msg='=' * 80),
        LogInfo(msg='WAREHOUSE ROBOT COMPLETE MAPPING SYSTEM'),
        LogInfo(msg='Hardware + Sensors + SLAM + Waypoint Recording'),
        LogInfo(msg='=' * 80),
        
        # Hardware
        control_node,
        robot_state_pub_node,
        
        # Delayed actions
        delay_joint_state_broadcaster,
        delay_mecanum_controller,
        delay_cmd_vel_bridge,
        delay_sensors,
        delay_slam,
        delay_waypoint_recorder,
        # delay_teleop,
    ]
    
    return LaunchDescription(declared_arguments + nodes)