#!/usr/bin/env python3
"""
Warehouse Rover Complete Launch File
Launches: USB Camera + QR Detector + Inventory Database
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments for camera configuration
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path (change to /dev/video2 if needed)'
    )
    
    camera_framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate'
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

    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'framerate': LaunchConfiguration('framerate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'pixel_format': 'yuyv',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap',
            'camera_name': 'usb_camera',
            # Camera controls
            'brightness': 110,
            'contrast': 150,
            'saturation': 60,
            'sharpness': 1,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Enhanced QR Detector Node
    qr_detector_node = Node(
        package='warehouse_rover_image_processing',
        executable='qr_detector_enhanced_node',
        name='qr_detector_enhanced',
        parameters=[{
            'enable_visualization': True,
            'enable_ipt': True,
            'enable_multipass': True,
            'target_width': LaunchConfiguration('image_width'),
            'target_height': LaunchConfiguration('image_height')
        }],
        remappings=[
            # Map QR detector input to USB camera output
            ('/camera/image_raw', '/image_raw'),
            ('/camera/camera_info', '/camera_info')
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Database/Inventory Node
    inventory_node = Node(
        package='warehouse_rover_database',
        executable='inventory_node',
        name='inventory',
        parameters=[{
            'database_path': '/tmp/warehouse_inventory.db',
            'auto_export': True,
            'export_dir': '/tmp/inventory_exports'
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        # Launch arguments
        video_device_arg,
        camera_framerate_arg,
        image_width_arg,
        image_height_arg,
        
        # Nodes
        usb_cam_node,
        qr_detector_node,
        inventory_node,
    ])