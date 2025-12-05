import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('warehouse_rover_lift_control')
    config_file = os.path.join(pkg_dir, 'config', 'lift_controller_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    lift_controller_node = Node(
        package='warehouse_rover_lift_control',
        executable='lift_controller_node',
        name='lift_controller',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        lift_controller_node
    ])
