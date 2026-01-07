from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the optical_flow_safety_node.
    """
    return LaunchDescription([
        Node(
            package='lift_safety',
            executable='optical_flow_node',
            name='optical_flow_safety_node',
            output='screen',
            parameters=[
                # {'video_topic': '/your/camera/topic'},
                # {'frame_width': 320},
                # {'stop_topic': '/lift/stop'},
                # {'magnitude_threshold': 12.0},
                # {'min_area': 800},
                # {'angle_tolerance': 25},
                # {'show_debug_windows': True},
            ]
        )
    ])
