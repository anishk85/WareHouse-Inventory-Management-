#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class AutoInitializeAMCL(Node):
    def __init__(self):
        super().__init__('auto_initialize_amcl')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        # Wait for system to stabilize
        time.sleep(2.0)
        self.publish_initial_pose()
        
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # CALCULATED from Gazebo position + map origin
        # Robot at Gazebo (0.799, 0.021) → Map origin (-2.52, -4.2)
        msg.pose.pose.position.x = 0.670  # 0.799 - (-2.52)
        msg.pose.pose.position.y = 0.143  # 0.021 - (-4.2)
        msg.pose.pose.position.z = 0.0

        # Robot facing WEST (-X direction, yaw ≈ -π)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -1.0
        msg.pose.pose.orientation.w = 0.0

        # Tight covariance for precise initialization
        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        self.publisher.publish(msg)
        self.get_logger().info(f'✅ AMCL initialized at ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}), facing WEST')
        time.sleep(1.0)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = AutoInitializeAMCL()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
