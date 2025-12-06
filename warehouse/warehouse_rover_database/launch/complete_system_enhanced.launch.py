from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB Camera (/dev/video2)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video2',
                'framerate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'usb_cam',
                'io_method': 'mmap'
            }],
            output='screen'
        ),
        
        # Enhanced QR Detector
        Node(
            package='warehouse_rover_image_processing',
            executable='qr_detector_enhanced_node',
            name='qr_detector_enhanced',
            parameters=[{
                'enable_visualization': True,
                'enable_ipt': True,
                'enable_multipass': True,
                'target_width': 640,
                'target_height': 480
            }],
            remappings=[
                ('/camera/image_raw', '/image_raw')
            ],
            output='screen'
        ),
        
        # Database/Inventory
        Node(
            package='warehouse_rover_database',
            executable='inventory_node',
            name='inventory',
            parameters=[{
                'database_path': '/tmp/warehouse_inventory.db',
                'auto_export': True,
                'export_dir': '/tmp/inventory_exports'
            }],
            output='screen'
        ),
    ])
