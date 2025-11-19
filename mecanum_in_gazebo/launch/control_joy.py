from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"dev": "/dev/input/js0"}],
        output="screen"
    )

    teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[{
            "axis_linear.x": 1,       # left stick up/down
            "axis_linear.y": 0,       # left stick left/right
            "axis_angular.yaw": 3,    # right stick left/right
            "scale_linear.x": 0.8,
            "scale_linear.y": 0.8,
            "scale_angular.yaw": 0.8,
        }],
        remappings=[
            ("/cmd_vel", "/cmd_vel")   # what your mecanum converter subscribes to
        ],
        output="screen"
    )

    return LaunchDescription([joy_node, teleop])
