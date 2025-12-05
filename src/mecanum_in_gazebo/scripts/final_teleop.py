#!/usr/bin/env python3
"""
Keyboard teleoperation node for mecanum robot (TwistStamped commands)
Controls: WASD for movement, QE for rotation, X to stop
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped  # <-- Changed from Twist
from rclpy.time import Time                 # <-- Added for timestamping

MAX_LINEAR_SPEED = 1.0    # m/s
MAX_ANGULAR_SPEED = 1.0   # rad/s
SPEED_INCREMENT = 0.1

class MecanumTeleopKey(Node):
    def __init__(self):
        super().__init__('mecanum_teleop_key')
        
        # Publisher - publish TwistStamped messages
        self.publisher = self.create_publisher(
            TwistStamped,  # <-- Changed to TwistStamped
            '/mecanum_drive_controller/reference',
            10
        )
        
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # --- FIX 1: Initialize speeds *before* calling print_usage ---
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        # ------------------------------------------------------------
        
        self.get_logger().info('Mecanum Teleop Node Started')
        self.print_usage() # Now this call is safe
        
        # Bindings
        self.movement_bindings = {
            'w': (1, 0, 0),    # Forward
            's': (-1, 0, 0),   # Backward
            'a': (0, 1, 0),    # Strafe left
            'd': (0, -1, 0),   # Strafe right
            'q': (0, 0, 1),    # Rotate left
            'e': (0, 0, -1),   # Rotate right
            'i': (1, 1, 0),    # Forward-left
            'o': (1, -1, 0),   # Forward-right
            'k': (-1, 1, 0),   # Backward-left
            'l': (-1, -1, 0),  # Backward-right
        }
        
        self.speed_bindings = {
            'z': 1.1,  # Increase speed
            'c': 0.9,  # Decrease speed
        }
        
        # Speeds are now initialized above
    
    def print_usage(self):
        msg = f"""
---------------------------
Mecanum Robot Teleop Control
---------------------------
Moving around:
   W         I    O
   A S D     K    L

W/S : Forward/Backward
A/D : Strafe Left/Right
Q/E : Rotate Left/Right
I/O/K/L : Diagonal movements

Z/C : Increase/Decrease speed by 10%
X / SPACE : Stop

Current linear speed: {self.linear_speed:.2f} m/s
Current angular speed: {self.angular_speed:.2f} rad/s
CTRL-C to quit
---------------------------
"""
        print(msg)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    # --- FIX 2 & 3: Publish TwistStamped and use msg.twist ---
    def publish_twist(self, linear_x, linear_y, angular_z):
        msg = TwistStamped()  # Create a TwistStamped message

        # Fill in the header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''  # You can leave this empty or use 'base_link'

        # Fill in the twist part
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.angular.z = angular_z
        
        self.publisher.publish(msg)
        self.get_logger().info(f"TwistStamped -> linear_x: {linear_x:.2f}, linear_y: {linear_y:.2f}, angular_z: {angular_z:.2f}")
    # ---------------------------------------------------------
    
    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key in self.movement_bindings:
                    lx, ly, az = self.movement_bindings[key]
                    self.publish_twist(
                        lx * self.linear_speed,
                        ly * self.linear_speed,
                        az * self.angular_speed
                    )
                    
                elif key in self.speed_bindings:
                    self.linear_speed *= self.speed_bindings[key]
                    self.angular_speed *= self.speed_bindings[key]
                    self.linear_speed = min(MAX_LINEAR_SPEED, max(0.1, self.linear_speed))
                    self.angular_speed = min(MAX_ANGULAR_SPEED, max(0.1, self.angular_speed))
                    self.get_logger().info(
                        f"Speeds updated -> linear: {self.linear_speed:.2f} m/s, angular: {self.angular_speed:.2f} rad/s"
                    )
                    
                elif key in ['x', 'X', ' ']:
                    # Stop
                    self.publish_twist(0.0, 0.0, 0.0)
                    self.get_logger().info('STOP')
                    
                elif key == '\x03':  # Ctrl+C
                    break
                    
        finally:
            # Always stop the robot on exit
            self.publish_twist(0.0, 0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = MecanumTeleopKey()
    try:
        teleop_node.run()
    except Exception as e:
        teleop_node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()