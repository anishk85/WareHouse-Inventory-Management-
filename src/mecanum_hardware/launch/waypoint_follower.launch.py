#!/usr/bin/env python3
"""
Waypoint Follower Launch File
Follows waypoints that were previously recorded during mapping

This launch file:
1. Starts hardware (Robot State Publisher, ROS2 Control, Mecanum Controller)
2. Loads the pre-recorded waypoints JSON file
3. Launches Nav2 with pre-loaded map
4. Starts waypoint following node
5. Visualizes path in RViz

Usage:
    # Default: uses latest waypoints file
    ros2 launch mecanum_hardware waypoint_follower.launch.py
    
    # Specify custom waypoints file
    ros2 launch mecanum_hardware waypoint_follower.launch.py \
        waypoints_file:=~/.ros/waypoints/waypoints_20251211_003037.json
    
    # Specify custom map
    ros2 launch mecanum_hardware waypoint_follower.launch.py \
        map_file:=/path/to/map.yaml
"""

import os
import glob
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def find_latest_waypoints_file(waypoint_dir):
    """Find the most recently modified waypoints file"""
    pattern = os.path.join(waypoint_dir, 'waypoints_*.json')
    files = glob.glob(pattern)
    if files:
        return max(files, key=os.path.getctime)
    return None


def generate_launch_description():
    # ==================== ARGUMENTS ====================
    declared_arguments = []
    
    # Default waypoints directory
    default_waypoint_dir = os.path.expanduser('~/.ros/waypoints')
    default_waypoints_file = find_latest_waypoints_file(default_waypoint_dir)
    if default_waypoints_file is None:
        default_waypoints_file = os.path.join(default_waypoint_dir, 'waypoints.json')
    
    declared_arguments.append(DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Use simulation (Gazebo) or real hardware'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'waypoints_file', default_value=default_waypoints_file,
        description='Path to waypoints JSON file'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'map_file', 
        default_value=PathJoinSubstitution([
            FindPackageShare('navigation_setup'),
            'maps',
            'maps5.yaml'
        ]),
        description='Path to map YAML file'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'nav2_enabled', default_value='true',
        description='Enable Nav2 for autonomous navigation'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'controllers_file', default_value='hardware_controllers.yaml',
        description='YAML file with the controllers configuration'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'description_package', default_value='mecanum_in_gazebo',
        description='Package with robot description'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'description_file', default_value='mec_rob.xacro',
        description='URDF/XACRO description file'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB2',
        description='Serial port for RPLidar'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'lidar_frame', default_value='lidar_link',
        description='Frame ID for LiDAR scan messages (matches URDF lidar.xacro)'))
    
    # Get configurations
    use_sim = LaunchConfiguration('use_sim')
    waypoints_file = LaunchConfiguration('waypoints_file')
    map_file = LaunchConfiguration('map_file')
    nav2_enabled = LaunchConfiguration('nav2_enabled')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_frame = LaunchConfiguration('lidar_frame')
    
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
    
    # 3. Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # 4. Mecanum Drive Controller Spawner
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # 5. CMD_VEL Bridge
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
    
    # ==================== LOCALIZATION & NAVIGATION ====================
    
    # 8. Map Server (serves pre-recorded map)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}],
        condition=IfCondition(nav2_enabled)
    )
    
    # 9. AMCL Localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            'min_particles': 500,
            'max_particles': 2000,
        }],
        condition=IfCondition(nav2_enabled)
    )
    
    # Lifecycle Manager to activate Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_nav2',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        condition=IfCondition(nav2_enabled)
    )
    
    # 10. Waypoint Follower Node (Custom)
    waypoint_follower_node = Node(
        package='mecanum_hardware',
        executable='waypoint_follower_node.py',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'goal_tolerance_linear': 0.1,
            'goal_tolerance_angular': 0.1,
            'max_linear_vel': 0.3,
            'max_angular_vel': 0.5,
        }],
        condition=IfCondition(nav2_enabled)
    )
    
    # 11. Path Visualizer Node (publishes RViz markers)
    path_visualizer_node = Node(
        package='mecanum_hardware',
        executable='waypoint_path_visualizer.py',
        name='waypoint_path_visualizer',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
        }],
        condition=IfCondition(nav2_enabled)
    )
    
    # ==================== DELAYED ACTIONS ====================
    
    # Delay joint_state_broadcaster 2 seconds
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
    
    # Delay mecanum_drive_controller 3 seconds
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
    
    # Delay cmd_vel_bridge 4 seconds
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
    
    # Delay sensors 5 seconds
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
    
    # Delay Nav2 stack 8 seconds (after sensors stabilize)
    delay_nav2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[
                        LogInfo(msg='---> Starting Navigation Stack (Map Server + AMCL)'),
                        lifecycle_manager,
                        map_server_node,
                        amcl_node,
                        path_visualizer_node,
                    ],
                )
            ],
        )
    )
    
    # Delay waypoint follower 12 seconds (after Nav2 is ready)
    delay_waypoint_follower = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=12.0,
                    actions=[
                        LogInfo(msg='---> Starting Waypoint Follower'),
                        LogInfo(msg=f'Loading waypoints from: {waypoints_file}'),
                        waypoint_follower_node
                    ],
                )
            ],
        )
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    
    nodes = [
        LogInfo(msg='=' * 80),
        LogInfo(msg='WAREHOUSE ROBOT WAYPOINT FOLLOWER'),
        LogInfo(msg='Hardware + Sensors + Localization + Waypoint Following'),
        LogInfo(msg='=' * 80),
        
        # Hardware
        control_node,
        robot_state_pub_node,
        
        # Delayed actions
        delay_joint_state_broadcaster,
        delay_mecanum_controller,
        delay_cmd_vel_bridge,
        delay_sensors,
        delay_nav2,
        delay_waypoint_follower,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
