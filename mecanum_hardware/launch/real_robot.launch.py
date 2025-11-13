"""
Complete launch file for real hardware with all sensors
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "esp1_port",
            default_value="/dev/ttyUSB0",
            description="ESP1 serial port (FL, FR motors)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "esp2_port",
            default_value="/dev/ttyUSB1",
            description="ESP2 serial port (BL, BR motors)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB2",
            description="LiDAR serial port",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sensor_fusion",
            default_value="true",
            description="Use robot_localization for sensor fusion",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            description="Launch LiDAR node",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Launch depth camera node",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            description="Launch IMU filter node",
        )
    )


    esp1_port = LaunchConfiguration("esp1_port")
    esp2_port = LaunchConfiguration("esp2_port")
    lidar_port = LaunchConfiguration("lidar_port")
    use_sensor_fusion = LaunchConfiguration("use_sensor_fusion")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    use_imu = LaunchConfiguration("use_imu")

    # use_sim=false menas hardware ros2 control
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mecanum_in_gazebo"), "urdf", "mec_rob.xacro"]
            ),
            " ",
            "use_sim:=false", 
            " ",
            "esp1_port:=",
            esp1_port,
            " ",
            "esp2_port:=",
            esp2_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # Controller manager configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mecanum_hardware"),
            "config",
            "controller.yaml",
        ]
    )

    # Controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/mecanum_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Mecanum drive controller spawner
    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay controller spawners after controller manager
    delay_joint_state_broadcaster_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_mecanum_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    # Robot localization (EKF) - sensor fusion
    ekf_config = PathJoinSubstitution(
        [FindPackageShare("mecanum_hardware"), "config", "ekf_localization.yaml"]
    )
    
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": False}],
        condition=IfCondition(use_sensor_fusion),
    )

    # LiDAR node (RPLidar)
    lidar_config = PathJoinSubstitution(
        [FindPackageShare("mecanum_hardware"), "config", "lidar.yaml"]
    )
    
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        parameters=[lidar_config, {"serial_port": lidar_port}],
        output="screen",
        condition=IfCondition(use_lidar),
    )

    # Depth Camera node (RealSense)
    camera_config = PathJoinSubstitution(
        [FindPackageShare("mecanum_hardware"), "config", "camera.yaml"]
    )
    
    camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        parameters=[camera_config],
        output="screen",
        condition=IfCondition(use_camera),
    )

    # IMU Filter node
    imu_config = PathJoinSubstitution(
        [FindPackageShare("mecanum_hardware"), "config", "imu.yaml"]
    )
    
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        parameters=[imu_config],
        output="screen",
        condition=IfCondition(use_imu),
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        delay_joint_state_broadcaster_after_control_node,
        delay_mecanum_controller_after_joint_state_broadcaster,
        robot_localization_node,
        lidar_node,
        camera_node,
        imu_filter_node,
    ]

    return LaunchDescription(declared_arguments + nodes)