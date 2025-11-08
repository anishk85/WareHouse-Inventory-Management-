"""
ROS2 launch for Gazebo with custom world and proper timing and environment setup
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    SetEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package Directories
    pkg_share = get_package_share_directory('mecanum_in_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'mec_rob.xacro')
    world_file = os.path.join(pkg_share, 'arena', 'arena.world')
    
    # Set Gazebo model path
    gazebo_models_path = os.path.join(pkg_share, 'meshes')
    arena_models_path = os.path.join(pkg_share, 'arena')
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_models_path + ':' + arena_models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    # Set Gazebo resource path
    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        pkg_share + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Declare launch arguments
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
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Gazebo server with custom world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'physics': 'ode'
        }.items()
    )
    
    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn robot in Gazebo with delay
    spawn_entity_node = TimerAction(
        period=3.0,
        actions=[
            Node(
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
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Set environment variables
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_resource_path)
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_entity_node)
    
    return ld