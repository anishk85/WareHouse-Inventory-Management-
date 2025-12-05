#!/usr/bin/env python3
"""
Lift Service Node - Provides ROS2 service interface for lift control
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import SetBool
from example_interfaces.srv import SetFloat
import time


class LiftServiceNode(Node):
    def __init__(self):
        super().__init__('lift_service_node')
        
        # Publisher for lift trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/lift_position_controller/joint_trajectory',
            10
        )
        
        # Service to move lift to specific height
        self.srv = self.create_service(
            SetFloat,
            'move_lift_to_height',
            self.move_lift_callback
        )
        
        self.current_position = 0.0
        
        self.get_logger().info('Lift Service Node started')
        self.get_logger().info('Service available: /move_lift_to_height')
        self.get_logger().info('Height range: 0.0 (down) to 1.0 (up)')
        
        time.sleep(0.5)  # Give publisher time to connect
    
    def move_lift_callback(self, request, response):
        """Service callback to move lift"""
        target_height = request.data
        
        # Validate height
        if not (0.0 <= target_height <= 1.0):
            self.get_logger().error(f'Invalid height: {target_height}. Must be 0.0-1.0')
            response.success = False
            response.message = 'Height out of range'
            return response
        
        # Move lift
        self.get_logger().info(f'Moving lift: {self.current_position:.2f}m → {target_height:.2f}m')
        
        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = ['lift_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [float(target_height)]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        traj.points = [point]
        
        # Publish
        self.traj_pub.publish(traj)
        
        # Wait for movement
        time.sleep(2.5)
        
        self.current_position = target_height
        self.get_logger().info(f'Lift reached {target_height:.2f}m')
        
        response.success = True
        response.message = f'Moved to {target_height}m'
        return response


def main():
    rclpy.init()
    
    try:
        node = LiftServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
