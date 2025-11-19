#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class VelocitySmoother(Node):
    def __init__(self):
        super().__init__('velocity_smoother')
        
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('rate', 50.0)
        
        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value
        rate = self.get_parameter('rate').value
        self.dt = 1.0 / rate
        
        self.current_cmd = Twist()
        self.target_cmd = Twist()
        
        self.sub = self.create_subscription(Twist, '/cmd_vel_in', self.cmd_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
        self.timer = self.create_timer(self.dt, self.smooth_callback)
    
    def cmd_callback(self, msg):
        self.target_cmd = msg
    
    def smooth_callback(self):
        smoothed = Twist()
        
        # Smooth linear velocities
        smoothed.linear.x = self._smooth_value(
            self.current_cmd.linear.x, self.target_cmd.linear.x, 
            self.max_lin_accel * self.dt
        )
        smoothed.linear.y = self._smooth_value(
            self.current_cmd.linear.y, self.target_cmd.linear.y,
            self.max_lin_accel * self.dt
        )
        
        # Smooth angular velocity
        smoothed.angular.z = self._smooth_value(
            self.current_cmd.angular.z, self.target_cmd.angular.z,
            self.max_ang_accel * self.dt
        )
        
        self.current_cmd = smoothed
        self.pub.publish(smoothed)
    
    def _smooth_value(self, current, target, max_change):
        diff = target - current
        if abs(diff) <= max_change:
            return target
        return current + math.copysign(max_change, diff)

def main():
    rclpy.init()
    node = VelocitySmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()