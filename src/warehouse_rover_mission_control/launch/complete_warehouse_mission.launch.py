from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        
        # QR Detection (lift camera)
        Node(
            package='warehouse_rover_image_processing',
            executable='qr_detector_enhanced_node',
            name='qr_detector_lift',
            remappings=[
                ('/camera/image_raw', '/lift_camera/image_raw'),
            ],
            output='screen'
        ),
        
        # Database
        Node(
            package='warehouse_rover_database',
            executable='inventory_node',
            name='inventory',
            output='screen'
        ),
        
        # Mission Controller
        Node(
            package='warehouse_rover_mission_control',
            executable='mission_controller_node',
            name='mission_controller',
            output='screen'
        ),
    ])
