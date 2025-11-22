#!/usr/bin/env python3
"""
CMD_VEL Bridge Node
Converts /cmd_vel (Twist) to /mecanum_drive_controller/reference (TwistStamped)
This is needed because most ROS2 nodes publish Twist on /cmd_vel,
but the mecanum_drive_controller expects TwistStamped on /reference
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # Subscribe to standard /cmd_vel (Twist)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to /mecanum_drive_controller/reference (TwistStamped)
        self.reference_pub = self.create_publisher(
            TwistStamped,
            '/mecanum_drive_controller/reference',
            10
        )
        
        self.get_logger().info('CMD_VEL Bridge started')
        self.get_logger().info('  Input: /cmd_vel (Twist)')
        self.get_logger().info('  Output: /mecanum_drive_controller/reference (TwistStamped)')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist to TwistStamped and republish"""
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        
        self.reference_pub.publish(stamped_msg)
        
        # Log occasionally for debugging
        if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(
                f'Bridging cmd_vel: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} wz={msg.angular.z:.2f}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
