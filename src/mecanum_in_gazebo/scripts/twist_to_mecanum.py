#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class TwistToMecanum(Node):
    def __init__(self):
        super().__init__("twist_to_mecanum")
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("lx_plus_ly", 0.275)

        self.r = self.get_parameter("wheel_radius").value
        self.l = self.get_parameter("lx_plus_ly").value

        self.pub = self.create_publisher(Float64MultiArray,
                                         "/mecanum_drive_controller/commands", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

    def cmd_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Mecanum inverse kinematics
        w_fl = (1/self.r) * (vx - vy - self.l * wz)
        w_fr = (1/self.r) * (vx + vy + self.l * wz)
        w_bl = (1/self.r) * (vx + vy - self.l * wz)
        w_br = (1/self.r) * (vx - vy + self.l * wz)

        arr = Float64MultiArray()
        arr.data = [w_fl, w_fr, w_bl, w_br]
        self.pub.publish(arr)
