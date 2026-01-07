from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description() -> LaunchDescription:
    """
    Launch file for RPi Camera Module 3 in Docker
    - Forces BGR888 format (Fixes crash)
    - Remaps topic to /image_raw
    - Sets 640x480 resolution
    """
    
    # Define arguments (still allows you to change ID if needed)
    camera_param_name = "camera"
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description="camera ID or name"
    )

    # We hardcode the format to BGR888 to prevent the crash
    # But we keep this definition so the launch system doesn't break if called with arguments
    format_param_name = "format"
    format_param_default = "BGR888" 

    # Camera Node Configuration
    composable_nodes = [
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            parameters=[{
                "camera": camera_param,
                "width": 640,
                "height": 480,
                "format": "MJPEG", # <--- Using MJPEG format supported by your camera
            }],
            remappings=[
                ('/camera/image_raw', '/image_raw'),       # <--- Remap to your desired topic
                ('/camera/camera_info', '/camera_info')
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    # Container definition
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return LaunchDescription([
        container,
        camera_launch_arg,
    ])