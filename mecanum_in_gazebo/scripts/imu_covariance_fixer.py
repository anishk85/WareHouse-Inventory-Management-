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
        
        # Subscribe to raw IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Publish fixed IMU
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data_fixed',
            10
        )
        
        # --- FIXED COVARIANCES ---
        
        # New orientation covariance (non-zero)
        # (0.01 std dev = ~5.7 deg)
        self.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # New angular velocity covariance (more realistic)
        # (0.01 std dev = ~0.57 deg/s)
        # The original 4.0e-8 was far too small.
        self.angular_velocity_covariance = [
            0.001, 0.0, 0.0,  # A small, reasonable variance
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        
        # --- END FIXED COVARIANCES ---
        
        self.get_logger().info('IMU Covariance Fixer started')
        self.get_logger().info('  Input: /imu/data')
        self.get_logger().info('  Output: /imu/data_fixed')
    
    def imu_callback(self, msg):
        """Fix orientation covariance and republish"""
        # Create new message (copy input)
        fixed_msg = Imu()
        fixed_msg.header = msg.header
        fixed_msg.orientation = msg.orientation
        fixed_msg.angular_velocity = msg.angular_velocity
        fixed_msg.linear_acceleration = msg.linear_acceleration
        
        # --- APPLY FIXED COVARIANCES ---
        
        # Override orientation covariance
        fixed_msg.orientation_covariance = self.orientation_covariance
        
        # Override angular velocity covariance
        fixed_msg.angular_velocity_covariance = self.angular_velocity_covariance
        
        # Keep the original linear acceleration covariance (we aren't fusing it)
        fixed_msg.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        
        # --- END APPLY FIXED COVARIANCES ---
        
        # Publish
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