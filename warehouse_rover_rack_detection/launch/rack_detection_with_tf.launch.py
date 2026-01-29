from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('warehouse_rover_rack_detection'),
        'config',
        'rack_detector_params.yaml'
    )
    
    return LaunchDescription([
        # Static TF: map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # Static TF: odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        
        # Rack Detector
        Node(
            package='warehouse_rover_rack_detection',
            executable='rack_detector_node',
            name='rack_detector',
            output='screen',
            parameters=[config]
        )
    ])
