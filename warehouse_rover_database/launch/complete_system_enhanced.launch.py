from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Enhanced QR Detector - USING LIFT CAMERA
        Node(
            package='warehouse_rover_image_processing',
            executable='qr_detector_enhanced_node',
            name='qr_detector_enhanced',
            parameters=[{
                'enable_visualization': True,
                'enable_ipt': True,
                'enable_multipass': True,
                'target_width': 1280,
                'target_height': 720
            }],
            remappings=[
                ('/camera/image_raw', '/lift_camera/image_raw')  # ← LIFT CAMERA!
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
