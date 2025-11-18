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
    

    urdf_file = os.path.join(pkg_share, 'urdf', 'mec_rob.xacro')
    world_file = os.path.join(pkg_share, 'arena', 'arena.world')
    controller_config = os.path.join(pkg_share, 'launch', 'controller.yaml')
    

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
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    rot_pose = LaunchConfiguration('rot_pose')
    

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
        default_value='+0.8',
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
    declare_rot_cmd=DeclareLaunchArgument(
        'rot_pose',
        default_value='3.14',
        description='Rotation of the robot'
    )
    

    robot_description = Command([
        'xacro ', urdf_file,
        ' controller_config_file:=', controller_config
    ])
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false'
        }.items()
    )
    

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mec_rob',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', rot_pose,
            '-timeout', '60.0'
        ],
        output='screen'
    )
    

    load_joint_broadcaster_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    

    load_mecanum_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    load_lift_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )


    # lidar_static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='lidar_static_tf',
    #     arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'lidar_link'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )
    
    # camera_static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_static_tf',
    #     arguments=['0.15', '0', '0.05', '0', '0', '0', 'base_link', 'camera_link'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )
    
    # imu_static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='imu_static_tf',
    #     arguments=['-0.05', '0', '0.02', '0', '0', '0', 'base_link', 'imu_link'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )
    

    ld = LaunchDescription()
    
    # Set environment variables
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_resource_path)
    
 
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_rot_cmd)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    # ld.add_action(TimerAction(period=2.0, actions=[lidar_static_tf]))
    # ld.add_action(TimerAction(period=2.0, actions=[camera_static_tf]))
    # ld.add_action(TimerAction(period=2.0, actions=[imu_static_tf]))

    ld.add_action(TimerAction(period=3.0, actions=[spawn_entity_cmd]))
    

    ld.add_action(TimerAction(period=5.0, actions=[load_joint_broadcaster_cmd]))
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_broadcaster_cmd,
                on_exit=[load_mecanum_controller_cmd]
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_mecanum_controller_cmd,
                on_exit=[load_lift_controller_cmd]
            )
        )
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[os.path.join(pkg_share, 'config', 'teleop_joy.yaml')],
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )


    
    return ld