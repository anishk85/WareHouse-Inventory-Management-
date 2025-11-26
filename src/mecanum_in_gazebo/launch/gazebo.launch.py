"""
ROS2 launch for Gazebo with proper mecanum drive control
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_in_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # File paths matching your existing structure
    urdf_file = os.path.join(pkg_share, 'urdf', 'mec_rob.xacro')
    world_file = os.path.join(pkg_share, 'arena', 'arena.world')
    controller_config = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    # Environment setup
    gazebo_models_path = os.path.join(pkg_share, 'meshes')
    arena_models_path = os.path.join(pkg_share, 'arena')
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_models_path + ':' + arena_models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        pkg_share + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world file to load'
    )
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot'
    )
    
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.055',
        description='Z position of the robot'
    )
    
    # Robot description with controller config passed to xacro
    robot_description = Command([
        'xacro ', urdf_file,
        ' use_sim:=true',
        ' controller_config_file:=', controller_config
    ])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false'
        }.items()
    )
    
    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn robot
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mec_rob',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-timeout', '60.0'
        ],
        output='screen'
    )
    
    # Load joint state broadcaster
    load_joint_broadcaster_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # Load mecanum drive controller with proper remappings
    # NOTE: mecanum_drive_controller uses /reference (TwistStamped), not /cmd_vel (Twist)
    # We'll create a bridge node to convert /cmd_vel -> /mecanum_drive_controller/reference
    load_mecanum_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # CMD_VEL Bridge - converts /cmd_vel to /mecanum_drive_controller/reference
    cmd_vel_bridge_node = Node(
        package='mecanum_in_gazebo',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # TF Odometry Relay - publishes odom->base_link TF to /tf
    # tf_odometry_relay_node = Node(
    #     package='mecanum_in_gazebo',
    #     executable='topic_bridge.py',
    #     name='tf_odometry_relay',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    # SENSOR Bridge - publishes mecanum odometry to /odom and fixes IMU covariances
    sensor_bridge_node = Node(
        package='mecanum_in_gazebo',
        executable='sensor_bridge.py',
        name='sensor_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Environment variables
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_resource_path)
    
    # Launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    
    # Nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    # Spawn robot after 3 seconds
    ld.add_action(TimerAction(period=3.0, actions=[spawn_entity_cmd]))
    
    # Load controllers after 7 seconds (wait for gazebo_ros2_control to start)
    ld.add_action(TimerAction(period=7.0, actions=[load_joint_broadcaster_cmd]))
    
    # Load mecanum controller 2 seconds after joint broadcaster
    ld.add_action(TimerAction(period=9.0, actions=[load_mecanum_controller_cmd]))
    
    # Start cmd_vel bridge after controller is loaded
    ld.add_action(TimerAction(period=10.0, actions=[cmd_vel_bridge_node]))
    
    # Start TF relay to publish odom->base_link transform to /tf
    # ld.add_action(TimerAction(period=10.0, actions=[tf_odometry_relay_node]))
    
    # Start sensor bridge to publish odometry to /odom
    ld.add_action(TimerAction(period=10.0, actions=[sensor_bridge_node]))
    
    return ld