#!/usr/bin/env python3
"""
Mecanum Drive Teleop Node
Publishes TwistStamped commands to mecanum_drive_controller/reference

Controls:
  w/s - forward/backward
  a/d - strafe left/right
  q/e - diagonal forward-left/forward-right
  z/c - diagonal backward-left/backward-right
  j/l - rotate left/right
  k - stop all motion
  +/- - increase/decrease speed
  r - reset to default speed
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty


class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('mecanum_teleop')
        
        # Publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/mecanum_drive_controller/reference',
            # 'twist_mux/cmd_vel_teleop',
            10
        )
        
        # Speed parameters
        self.linear_speed = 0.3  # m/s
        self.strafe_speed = 0.3  # m/s (lateral motion)
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.05
        
        # Key bindings
        self.move_bindings = {
            'w': (1, 0, 0),    # forward
            's': (-1, 0, 0),   # backward
            'a': (0, 1, 0),    # strafe left
            'd': (0, -1, 0),   # strafe right
            'q': (1, 1, 0),    # forward-left
            'e': (1, -1, 0),   # forward-right
            'z': (-1, 1, 0),   # backward-left
            'c': (-1, -1, 0),  # backward-right
            'j': (0, 0, 1),    # rotate left
            'l': (0, 0, -1),   # rotate right
        }
        
        self.speed_bindings = {
            '+': 1.1,
            '=': 1.1,
            '-': 0.9,
            '_': 0.9,
        }
        
        # Terminal settings
        self.settings = None
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('   Mecanum Drive Teleop Node Started   ')
        self.get_logger().info('═══════════════════════════════════════')
        self.print_instructions()
    
    def print_instructions(self):
        """Print control instructions"""
        msg = """
╔════════════════════════════════════════════════════════╗
║           MECANUM DRIVE TELEOP CONTROLS                ║
╠════════════════════════════════════════════════════════╣
║                                                        ║
║  LINEAR MOTION:                DIAGONAL MOTION:       ║
║    w - Forward                   q - Forward Left     ║
║    s - Backward                  e - Forward Right    ║
║    a - Strafe Left               z - Backward Left    ║
║    d - Strafe Right              c - Backward Right   ║
║                                                        ║
║  ROTATION:                     SPEED CONTROL:         ║
║    j - Rotate Left               + - Increase speed   ║
║    l - Rotate Right              - - Decrease speed   ║
║                                  r - Reset speed      ║
║                                                        ║
║  STOP:                         INFO:                  ║
║    k - Stop All Motion           i - Print status     ║
║    SPACE - Stop All              h - Help             ║
║                                                        ║
║  Ctrl+C - Exit                                        ║
╚════════════════════════════════════════════════════════╝

Current speeds: Linear={:.2f} m/s, Strafe={:.2f} m/s, Angular={:.2f} rad/s
        """.format(self.linear_speed, self.strafe_speed, self.angular_speed)
        print(msg)
    
    def get_key(self):
        """Get keyboard input (blocking)"""
        if sys.stdin.isatty():
            tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_twist(self, x, y, th):
        """Publish TwistStamped message"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = x * self.linear_speed
        msg.twist.linear.y = y * self.strafe_speed
        msg.twist.linear.z = 0.0
        
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = th * self.angular_speed
        
        self.cmd_pub.publish(msg)
        
        # Log the movement
        if x != 0 or y != 0 or th != 0:
            motion = []
            if x > 0:
                motion.append("Forward")
            elif x < 0:
                motion.append("Backward")
            
            if y > 0:
                motion.append("Strafe Left")
            elif y < 0:
                motion.append("Strafe Right")
            
            if th > 0:
                motion.append("Rotate Left")
            elif th < 0:
                motion.append("Rotate Right")
            
            self.get_logger().info(f'→ {" + ".join(motion)} | Vx={x*self.linear_speed:.2f} Vy={y*self.strafe_speed:.2f} Vth={th*self.angular_speed:.2f}')
    
    def run(self):
        """Main run loop"""
        x = 0
        y = 0
        th = 0
        
        try:
            while True:
                key = self.get_key()
                
                if key in self.move_bindings.keys():
                    x = self.move_bindings[key][0]
                    y = self.move_bindings[key][1]
                    th = self.move_bindings[key][2]
                    self.publish_twist(x, y, th)
                    
                elif key in self.speed_bindings.keys():
                    self.linear_speed *= self.speed_bindings[key]
                    self.strafe_speed *= self.speed_bindings[key]
                    self.angular_speed *= self.speed_bindings[key]
                    
                    self.get_logger().info(f'Speed: Linear={self.linear_speed:.2f} m/s, Strafe={self.strafe_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s')
                    self.publish_twist(x, y, th)
                    
                elif key == 'r':
                    # Reset speed
                    self.linear_speed = 0.3
                    self.strafe_speed = 0.3
                    self.angular_speed = 1.0
                    self.get_logger().info('Speed reset to defaults')
                    self.publish_twist(x, y, th)
                    
                elif key == ' ' or key == 'k':
                    # Stop
                    x = 0
                    y = 0
                    th = 0
                    self.publish_twist(x, y, th)
                    self.get_logger().info('→ STOPPED')
                    
                elif key == 'i':
                    # Print status
                    print(f"\n┌────────────────────────────────────┐")
                    print(f"│ Current Speeds:                    │")
                    print(f"│  Linear:  {self.linear_speed:.2f} m/s              │")
                    print(f"│  Strafe:  {self.strafe_speed:.2f} m/s              │")
                    print(f"│  Angular: {self.angular_speed:.2f} rad/s            │")
                    print(f"└────────────────────────────────────┘\n")
                    
                elif key == 'h':
                    self.print_instructions()
                    
                elif key == '\x03':  # Ctrl+C
                    break
                
                else:
                    # Any other key stops the robot
                    if x != 0 or y != 0 or th != 0:
                        x = 0
                        y = 0
                        th = 0
                        self.publish_twist(x, y, th)
                        
        except KeyboardInterrupt:
            pass
        
        finally:
            # Send final stop command
            self.publish_twist(0, 0, 0)
            
            if sys.stdin.isatty():
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            self.get_logger().info('Shutting down teleop...')


def main(args=None):
    rclpy.init(args=args)
    
    node = MecanumTeleop()
    
    try:
        node.run()
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()