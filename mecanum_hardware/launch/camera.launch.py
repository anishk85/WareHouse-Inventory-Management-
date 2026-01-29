#!/usr/bin/env python3
"""
USB Camera Launch File for ROS2 Humble
Save as: ~/ros2_ws/src/camera_launch.py
Usage: ros2 launch camera_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate'
    )

    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_camera',
        namespace='',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'pixel_format': 'yuyv',
            'framerate': LaunchConfiguration('framerate'),
            'camera_name': 'usb_camera',
            'io_method': 'mmap',
            'camera_info_url': 'file:///root/.ros/camera_info/usb_camera.yaml',
            # Control settings
            'brightness': 110,
            'contrast': 150,
            'saturation': 60,
            'sharpness': 1,
            'gain': 16,
            'auto_white_balance': True,
        }],
        output='screen',
        respawn=True,  # Auto-restart if it crashes
        respawn_delay=2.0
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        usb_cam_node,
    ])