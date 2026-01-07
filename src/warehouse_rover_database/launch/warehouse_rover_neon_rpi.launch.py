#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════════════╗
║  🚀 Warehouse Rover - RPi Camera + Neon PostgreSQL Edition           ║
╠═══════════════════════════════════════════════════════════════════════╣
║  📷 Camera: Raspberry Pi Camera Module 3 (camera_ros)                ║
║  🔍 QR Detection: Enhanced multi-pass detector                        ║
║  💾 Database: Neon PostgreSQL (Serverless Cloud Database)             ║
╠═══════════════════════════════════════════════════════════════════════╣
║  Topic Flow:                                                          ║
║    📸 RPi Camera -> /image_raw                                        ║
║         ↓                                                             ║
║    🔍 QR Detector -> /qr_detections -> /qr_detection_viz             ║
║         ↓                                                             ║
║    💾 Inventory (Neon DB) -> ☁️ Cloud Storage                         ║
╚═══════════════════════════════════════════════════════════════════════╝
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ============================================================
    # 📋 Launch Arguments
    # ============================================================
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='📐 Image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='📐 Image height'
    )
    
    rpi_camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='🎥 Camera ID for RPi Camera Module'
    )

    # ============================================================
    # 📷 RPi Camera Module 3 Node
    # ============================================================
    
    rpi_camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera_node',
                parameters=[{
                    'camera': LaunchConfiguration('camera_id'),
                    'width': LaunchConfiguration('image_width'),
                    'height': LaunchConfiguration('image_height'),
                    'format': 'YUYV', 
                }],
                remappings=[
                    # Remap to standard /image_raw topic
                    ('/camera_node/image_raw', '/image_raw'),
                    ('/camera_node/camera_info', '/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # ============================================================
    # 🔍 QR Detection Node
    # ============================================================
    
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
            ('/camera/image_raw', '/image_raw'),
            ('/camera/camera_info', '/camera_info')
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # ============================================================
    # 💾 Database/Inventory Node (Neon PostgreSQL)
    # ============================================================
    
    # Path to Neon parameters YAML file
    neon_params_file = PathJoinSubstitution([
        FindPackageShare('warehouse_rover_database'),
        'config',
        'neon_params.yaml'
    ])
    
    inventory_node = Node(
        package='warehouse_rover_database',
        executable='inventory_node_neon.py',
        name='inventory_node_neon',
        parameters=[neon_params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        # 📋 Arguments
        image_width_arg,
        image_height_arg,
        rpi_camera_id_arg,
        # 🚀 Nodes
        rpi_camera_container,   # 📷 RPi Camera
        qr_detector_node,       # 🔍 QR Detection
        inventory_node,         # 💾 Neon DB
    ])