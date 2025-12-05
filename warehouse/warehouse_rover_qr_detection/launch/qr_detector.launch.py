import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('warehouse_rover_qr_detection')
    
    # Parameter file
    params_file = os.path.join(pkg_dir, 'config', 'qr_detector_params.yaml')
    
    # QR Detector Node
    qr_detector_node = Node(
        package='warehouse_rover_qr_detection',
        executable='qr_detector_node',
        name='qr_detector_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
        ]
    )
    
    return LaunchDescription([
        qr_detector_node,
    ])
