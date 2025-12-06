#!/usr/bin/env python3
"""
IMU Covariance Fixer
Republishes IMU data with proper orientation AND angular velocity covariance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceFixer(Node):
    def __init__(self):
        super().__init__('imu_covariance_fixer')

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data_fixed',
            10
        )
        
        self.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        

        self.angular_velocity_covariance = [
            0.001, 0.0, 0.0,  # A small, reasonable variance
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        

        
        self.get_logger().info('IMU Covariance Fixer started')
        self.get_logger().info('  Input: /imu/data')
        self.get_logger().info('  Output: /imu/data_fixed')
    
    def imu_callback(self, msg):
        """Fix orientation covariance and republish"""
        fixed_msg = Imu()
        fixed_msg.header = msg.header
        fixed_msg.orientation = msg.orientation
        fixed_msg.angular_velocity = msg.angular_velocity
        fixed_msg.linear_acceleration = msg.linear_acceleration
        
        fixed_msg.orientation_covariance = self.orientation_covariance
        
        fixed_msg.angular_velocity_covariance = self.angular_velocity_covariance
        
        fixed_msg.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        
        self.imu_pub.publish(fixed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()