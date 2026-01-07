from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Update this path to where you saved your yaml file
    config_file = os.path.join(
        get_package_share_directory('mecanum_hardware'),
        'config',
        'box_filters.yaml'
    )

    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[config_file],
            remappings=[
                # Input: The raw topic from your LiDAR driver
                ('scan', '/scan_raw'), 
                # Output: The clean topic Nav2 and Cartographer will use
                ('scan_filtered', '/scan') 
            ]
        )
    ])