#!/usr/bin/env python3
"""
Convert cmd_vel (Twist) to mecanum wheel velocities
Subscribes: /cmd_vel (geometry_msgs/Twist)
Publishes: /mecanum_drive_controller/commands (std_msgs/Float64MultiArray)
"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math




class CmdVelToMecanum(Node):
    
    
    def __init__(self):
        super().__init__('cmd_vel_to_mecanum')
        
        # Declare parameters
        self.declare_parameter('wheel_separation_x', 0.3)   # front-back distance
        self.declare_parameter('wheel_separation_y', 0.25)  # left-right distance
        self.declare_parameter('wheel_radius', 0.05)        # wheel radius
        
        # Get parameters
        self.wheel_sep_x = self.get_parameter('wheel_separation_x').value
        self.wheel_sep_y = self.get_parameter('wheel_separation_y').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Publishers and Subscribers
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/mecanum_drive_controller/commands',
            10
        )
        
        self.get_logger().info('cmd_vel to mecanum converter started')
        self.get_logger().info(f'Wheel params: sep_x={self.wheel_sep_x}, sep_y={self.wheel_sep_y}, radius={self.wheel_radius}')
        

    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist (vx, vy, wz) to mecanum wheel velocities
        Mecanum kinematics:
        v_fl = (vx - vy - wz*(lx+ly)) / r
        v_fr = (vx + vy + wz*(lx+ly)) / r
        v_bl = (vx + vy - wz*(lx+ly)) / r
        v_br = (vx - vy + wz*(lx+ly)) / r
        """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Half distances
        lx = self.wheel_sep_x / 2.0
        ly = self.wheel_sep_y / 2.0
        
        # Compute wheel velocities (rad/s)
        v_fl = (vx - vy - wz * (lx + ly)) / self.wheel_radius
        v_fr = (vx + vy + wz * (lx + ly)) / self.wheel_radius
        v_bl = (vx + vy - wz * (lx + ly)) / self.wheel_radius
        v_br = (vx - vy + wz * (lx + ly)) / self.wheel_radius
        
        # Publish wheel commands
        wheel_cmd = Float64MultiArray()
        wheel_cmd.data = [v_fl, v_fr, v_bl, v_br]
        
        self.publisher.publish(wheel_cmd)
        
        # Log occasionally
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
            self.get_logger().info(
                f'cmd_vel: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} -> '
                f'wheels: FL={v_fl:.2f} FR={v_fr:.2f} BL={v_bl:.2f} BR={v_br:.2f}',
                throttle_duration_sec=1.0
            )




def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMecanum()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()