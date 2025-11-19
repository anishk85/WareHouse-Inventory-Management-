#!/usr/bin/env python3
"""
Simple Lift Controller - Fixed for Segfault
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import time

class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller_node')
        
        # Publisher for trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/lift_position_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Lift Controller initialized')
        time.sleep(0.5)  # Give publisher time to connect
    
    def move_lift(self, position):
        """Move lift to target position"""
        
        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = ['lift_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        traj.points = [point]
        
        # Publish
        self.get_logger().info(f'Moving lift to: {position}')
        self.traj_pub.publish(traj)
        self.get_logger().info('Trajectory sent!')


def main():
    rclpy.init()
    
    try:
        controller = LiftController()
        
        if len(sys.argv) > 1:
            position = float(sys.argv[1])
            if 0.0 <= position <= 1.0:
                controller.move_lift(position)
                time.sleep(3)
            else:
                print("Error: Position must be between 0.0 and 1.0")
        else:
            print("\nUsage: python3 lift_action_client.py <position>")
            print("Example: python3 lift_action_client.py 0.5")
            print("Position range: 0.0 (down) to 1.0 (up)")
        
        rclpy.shutdown()
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
