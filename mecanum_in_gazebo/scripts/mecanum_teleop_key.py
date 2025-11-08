#!/usr/bin/env python3
"""
Keyboard teleoperation node for mecanum robot
Controls: WASD for movement, QE for rotation, X to stop
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MecanumTeleopKey(Node):
    def __init__(self):
        super().__init__('mecanum_teleop_key')
        
        # Publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.5    # m/s
        self.angular_speed = 1.0   # rad/s
        self.speed_increment = 0.1
        
        # Key bindings
        self.movement_bindings = {
            'w': (1, 0, 0),    # Forward
            's': (-1, 0, 0),   # Backward
            'a': (0, 1, 0),    # Left (strafe)
            'd': (0, -1, 0),   # Right (strafe)
            'q': (0, 0, 1),    # Rotate left
            'e': (0, 0, -1),   # Rotate right
            # Diagonal movements
            'i': (1, 1, 0),    # Forward-left
            'o': (1, -1, 0),   # Forward-right
            'k': (-1, 1, 0),   # Backward-left
            'l': (-1, -1, 0),  # Backward-right
        }
        
        self.speed_bindings = {
            'z': (1.1, 1.1),   # Increase speed
            'c': (0.9, 0.9),   # Decrease speed
        }
        
        self.get_logger().info('Mecanum Teleop Node Started')
        self.print_usage()
        
    def print_usage(self):
        msg = """
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
X   : Force stop
SPACE : Stop

Current speeds:
  Linear: {:.2f} m/s
  Angular: {:.2f} rad/s

CTRL-C to quit
---------------------------
        """.format(self.linear_speed, self.angular_speed)
        print(msg)
    
    def get_key(self):
        """Get keyboard input (non-blocking)"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_twist(self, x, y, theta):
        """Publish Twist message"""
        twist = Twist()
        twist.linear.x = x * self.linear_speed
        twist.linear.y = y * self.linear_speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = theta * self.angular_speed
        
        self.publisher.publish(twist)
        
        # Log the command
        self.get_logger().info(
            f'Publishing: linear(x={twist.linear.x:.2f}, y={twist.linear.y:.2f}), '
            f'angular(z={twist.angular.z:.2f})'
        )
    
    def run(self):
        """Main loop"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key in self.movement_bindings:
                    x, y, theta = self.movement_bindings[key]
                    self.publish_twist(x, y, theta)
                    
                elif key in self.speed_bindings:
                    linear_mult, angular_mult = self.speed_bindings[key]
                    self.linear_speed *= linear_mult
                    self.angular_speed *= angular_mult
                    
                    self.get_logger().info(
                        f'Speed updated: linear={self.linear_speed:.2f} m/s, '
                        f'angular={self.angular_speed:.2f} rad/s'
                    )
                    self.publish_twist(0, 0, 0)  # Stop after speed change
                    
                elif key in ['x', 'X', ' ']:
                    # Emergency stop
                    self.publish_twist(0, 0, 0)
                    self.get_logger().info('STOP')
                    
                elif key == '\x03':  # Ctrl+C
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            
        finally:
            # Send stop command before exit
            self.publish_twist(0, 0, 0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = MecanumTeleopKey()
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()