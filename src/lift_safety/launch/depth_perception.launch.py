from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the depth_perception_node.
    """
    return LaunchDescription([
        Node(
            package='lift_safety',
            executable='depth_perception_node',
            name='depth_perception_safety_node',
            output='screen',
            parameters=[
                # {'video_topic': '/your/camera/topic'},
                # {'stop_topic': '/lift/stop'},
                # {'model_id': 'depth-anything/Depth-Anything-V2-Small-hf'},
                # {'process_width': 480},
                # {'process_height': 360},
                # {'collision_threshold': 230},
                # {'roi_size': 300},
                # {'show_debug_windows': True},
            ]
        )
    ])
