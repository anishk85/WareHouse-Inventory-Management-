#!/usr/bin/env python3
"""
Launch file for MongoDB-based inventory system
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('warehouse_rover_database')
    params_file = os.path.join(pkg_dir, 'config', 'mongodb_params.yaml')
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'mongodb_connection_string',
            default_value='',
            description='MongoDB Atlas connection string (or set MONGODB_CONNECTION_STRING env var)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_local',
            default_value='false',
            description='Use local MongoDB instead of Atlas'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'export_dir',
            default_value='/tmp/inventory_exports',
            description='Directory for mission exports'
        )
    )
    
    # Inventory node
    inventory_node = Node(
        package='warehouse_rover_database',
        executable='inventory_node_mongo.py',
        name='inventory_node',
        output='screen',
        parameters=[
            params_file,
            {
                'mongodb_connection_string': LaunchConfiguration('mongodb_connection_string'),
                'use_local_mongodb': LaunchConfiguration('use_local'),
                'export_dir': LaunchConfiguration('export_dir')
            }
        ]
    )
    
    return LaunchDescription(declared_arguments + [
        inventory_node
    ])
