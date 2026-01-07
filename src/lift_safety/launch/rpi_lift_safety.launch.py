#!/usr/bin/env python3
"""
RPi4 Lift Safety Launch File
Runs on RPi4 connected to camera
Publishes safety signals to RPi5 via ROS network (ROS_DOMAIN_ID)

Components:
- Optical Flow Safety (always enabled)
- Depth Estimation Safety (optional, resource intensive)
- Camera driver

Setup:
1. Set ROS_DOMAIN_ID to same value on both RPis
2. Set ROS_LOCALHOST_ONLY=0
3. Verify network connectivity
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== ARGUMENTS ====================
    declared_arguments = []
    
    declared_arguments.append(DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'stop_topic',
        default_value='/lift/stop',
        description='Topic for stop/collision signals'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'enable_depth_estimation',
        default_value='false',
        description='Enable depth estimation (resource intensive)'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'show_debug_windows',
        default_value='false',
        description='Show debug windows (set false for headless)'))
    
    # Get configurations
    camera_topic = LaunchConfiguration('camera_topic')
    camera_device = LaunchConfiguration('camera_device')
    stop_topic = LaunchConfiguration('stop_topic')
    enable_depth_estimation = LaunchConfiguration('enable_depth_estimation')
    show_debug_windows = LaunchConfiguration('show_debug_windows')
    
    # ==================== NODES ====================
    
    # 1. Camera Driver (USB camera)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'framerate': 30.0
        }],
        remappings=[
            ('image_raw', camera_topic)
        ]
    )
    
    # 2. Optical Flow Safety (Always Enabled)
    optical_flow_node = Node(
        package='lift_safety',
        executable='optical_flow_node',
        name='optical_flow_safety',
        output='screen',
        parameters=[{
            'video_topic': camera_topic,
            'stop_topic': stop_topic,
            'frame_width': 320,
            'magnitude_threshold': 12.0,
            'min_area': 800,
            'angle_tolerance': 25,
            'show_debug_windows': show_debug_windows
        }]
    )
    
    # 3. Depth Estimation Safety (Optional)
    depth_estimation_node = Node(
        package='lift_safety',
        executable='depth_perception_node',
        name='depth_perception_safety',
        output='screen',
        parameters=[{
            'video_topic': camera_topic,
            'stop_topic': stop_topic,
            'model_id': 'depth-anything/Depth-Anything-V2-Small-hf',
            'process_width': 480,
            'process_height': 360,
            'collision_threshold': 230,
            'roi_size': 300,
            'show_debug_windows': show_debug_windows
        }],
        condition=IfCondition(enable_depth_estimation)
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    
    nodes = [
        camera_node,
        optical_flow_node,
        depth_estimation_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)