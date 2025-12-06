#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    ld = LaunchDescription()
    
    load_joint_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    load_mecanum_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    
    ld.add_action(TimerAction(period=5.0, actions=[load_joint_broadcaster_cmd]))
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_broadcaster_cmd,
                on_exit=[load_mecanum_controller_cmd],
            )
        )
    )
    
    print("Joint Broadcaster and Mecanum Controller Launch File Executed")
    
    return ld
              
