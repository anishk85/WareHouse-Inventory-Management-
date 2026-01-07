#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════════════╗
║  🚀 Warehouse Rover Complete System - Neon PostgreSQL Edition        ║
╠═══════════════════════════════════════════════════════════════════════╣
║  📷 Camera: USB Camera (usb_cam)                                      ║
║  🔍 QR Detection: Enhanced multi-pass detector                        ║
║  💾 Database: Neon PostgreSQL (Serverless Cloud Database)             ║
╠═══════════════════════════════════════════════════════════════════════╣
║  Topic Flow:                                                          ║
║    📸 USB Camera -> /image_raw                                        ║
║         ↓                                                             ║
║    🔍 QR Detector -> /qr_detections -> /qr_detection_viz             ║
║         ↓                                                             ║
║    💾 Inventory (Neon DB) -> Cloud Storage                            ║
╚═══════════════════════════════════════════════════════════════════════╝
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ============================================================
    # Launch Arguments
    # ============================================================
    
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path for USB camera (try /dev/video0 or /dev/video2)'
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
    
    # ============================================================
    # USB Camera Node (UNCHANGED - Working Configuration)
    # ============================================================
    # Publishes to: /image_raw, /camera_info
    
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
            # Camera controls (adjust based on your camera)
            'brightness': 110,
            'contrast': 150,
            'saturation': 60,
            'sharpness': 1,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # ============================================================
    # QR Detection Node (UNCHANGED - Working Configuration)
    # ============================================================
    
    # Enhanced QR Detector Node
    # Subscribes to: /image_raw (from camera)
    # Publishes to: /qr_detections, /qr_detection_viz
    qr_detector_node = Node(
        package='warehouse_rover_image_processing',
        executable='qr_detector_enhanced_node',
        name='qr_detector_enhanced',
        parameters=[{
            'enable_visualization': True,
            'enable_ipt': True,              # Image Processing Techniques
            'enable_multipass': True,         # Multiple detection passes
            'target_width': LaunchConfiguration('image_width'),
            'target_height': LaunchConfiguration('image_height')
        }],
        remappings=[
            # Subscribe to the standard /image_raw topic from camera
            ('/camera/image_raw', '/image_raw'),
            ('/camera/camera_info', '/camera_info')
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # ============================================================
    # Database/Inventory Node (UPDATED - Now using Neon PostgreSQL)
    # ============================================================
    
    # Path to Neon parameters YAML file
    neon_params_file = PathJoinSubstitution([
        FindPackageShare('warehouse_rover_database'),
        'config',
        'neon_params.yaml'
    ])
    
    # Inventory Node with Neon PostgreSQL (CHANGED FROM MONGODB)
    # Subscribes to: /qr_detections
    # Stores data in Neon cloud database (serverless PostgreSQL)
    # Configuration loaded from neon_params.yaml
    inventory_node = Node(
        package='warehouse_rover_database',
        executable='inventory_node_neon.py',  # Changed from inventory_node_mongo.py
        name='inventory_node_neon',            # Changed from inventory_node_mongo
        parameters=[neon_params_file],         # Changed from mongodb_params_file
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        # ============================================================
        # Launch Arguments
        # ============================================================
        video_device_arg,
        camera_framerate_arg,
        image_width_arg,
        image_height_arg,
        
        # ============================================================
        # Nodes
        # ============================================================
        usb_cam_node,          # USB Camera (unchanged)
        qr_detector_node,      # QR Detection (unchanged)
        inventory_node,        # Neon PostgreSQL Inventory (UPDATED)
    ])
