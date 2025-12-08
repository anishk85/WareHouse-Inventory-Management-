import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('warehouse_rover_database')
        
    inventory_node = Node(
        package='warehouse_rover_database',
        executable='inventory_node',
        name='inventory_node',
        output='screen',
        parameters=[
            params_file,
            {'rack_map_file': rack_map_file}
        ]
    )
    
    return LaunchDescription([inventory_node])
