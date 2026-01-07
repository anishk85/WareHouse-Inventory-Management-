"""Simple launch file for USB camera on Raspberry Pi 4."""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with configurable parameters."""
    
    # Get package directory
    try:
        pkg_dir = get_package_share_directory('usb_cam')
    except:
        pkg_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'usb_cam')
    
    # Default parameter file path
    default_params_file = os.path.join(pkg_dir, 'config', 'params_1.yaml')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the parameter file to use'
    )
    
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device (e.g., /dev/video0)'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera1',
        description='Camera name'
    )

    # Camera node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=LaunchConfiguration('camera_name'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'video_device': LaunchConfiguration('video_device')}
        ],
    )

    return LaunchDescription([
        params_file_arg,
        video_device_arg,
        camera_name_arg,
        camera_node
    ])