#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped  
import sys
import termios
import tty
import threading  

msg = """
---------------------------
Mecanum Robot Teleop Control (Continuous Version)
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/,  : forward/backward (x-axis)
j/l  : strafe left/right (y-axis)
J/L  : rotate left/right

u : forward + strafe left
o : forward + strafe right
m : backward + strafe left
. : backward + strafe right

k : stop (or any other unmapped key)
SPACE : emergency stop

t/b : increase/decrease max speeds by 10%
y/n : increase/decrease only linear speed by 10%
h/; : increase/decrease only angular speed by 10%

CTRL-C to quit
---------------------------
Current Settings:
"""

# Updated moveBindings to use J/L for rotation
moveBindings = {
    'i': (1, 0, 0, 0),    # forward
    ',': (-1, 0, 0, 0),   # backward
    'j': (0, 1, 0, 0),    # strafe left
    'l': (0, -1, 0, 0),   # strafe right
    'u': (1, 1, 0, 0),    # forward-left
    'o': (1, -1, 0, 0),   # forward-right
    'm': (-1, 1, 0, 0),   # backward-left
    '.': (-1, -1, 0, 0),  # backward-right
    'J': (0, 0, 0, 1),    # rotate left
    'L': (0, 0, 0, -1),   # rotate right
    'k': (0, 0, 0, 0),    # stop
}

speedBindings = {
    't': (1.1, 1.1),  # increase all speeds
    'b': (0.9, 0.9),  # decrease all speeds
    'y': (1.1, 1.0),  # increase linear speed
    'n': (0.9, 1.0),  # decrease linear speed
    'h': (1.0, 1.1),  # increase angular speed
    ';': (1.0, 0.9),  # decrease angular speed
}

# Store settings globally for reuse
settings = termios.tcgetattr(sys.stdin)

def getKey():
    # This function is blocking, which is what we want for the key_loop thread
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tlinear speed: %.2f\tangular speed: %.2f " % (speed, turn)

class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('mecanum_teleop')
        
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/mecanum_drive_controller/reference',
            10
        )
        
        self.speed = 0.3
        self.turn = 0.5
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        
        self.speed_limit = 0.6
        self.turn_limit = 1.0

        self.lock = threading.Lock()
        
        self.get_logger().info('Mecanum Teleop Node Started (Continuous Version)')
        self.get_logger().info(msg)
        self.get_logger().info(vels(self.speed, self.turn))

        self.publish_timer = self.create_timer(
            0.1,  # Publish at 10 Hz
            self.publish_loop
        )

        self.key_thread = threading.Thread(target=self.key_loop)
        self.key_thread.daemon = True  # Exit when main thread exits
        self.key_thread.start()

    def key_loop(self):
        while rclpy.ok():
            try:
                key = getKey()
                
                if key == '\x03':  # CTRL-C
                    self.get_logger().info('CTRL-C pressed, shutting down...')
                    rclpy.shutdown()
                    break
                
                x, y, z, th = 0.0, 0.0, 0.0, 0.0

                if key in moveBindings.keys():
                    x = float(moveBindings[key][0])
                    y = float(moveBindings[key][1])
                    z = float(moveBindings[key][2])
                    th = float(moveBindings[key][3])
                    
                elif key in speedBindings.keys():
                    with self.lock:
                        self.speed = min(self.speed * speedBindings[key][0], self.speed_limit)
                        self.turn = min(self.turn * speedBindings[key][1], self.turn_limit)
                        self.speed = max(0.1, self.speed)  # Enforce min
                        self.turn = max(0.1, self.turn)    # Enforce min
                    self.get_logger().info(vels(self.speed, self.turn))
                    continue

                elif key == ' ':
                    self.get_logger().info('EMERGENCY STOP')
                

                with self.lock:
                    self.x = x
                    self.y = y
                    self.z = z
                    self.th = th

            except Exception as e:
                self.get_logger().error(f'Error in key_loop: {e}')
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


    def publish_loop(self):
        with self.lock:
            x_cmd = self.x * self.speed
            y_cmd = self.y * self.speed
            z_cmd = self.z * self.speed
            th_cmd = self.th * self.turn

        stamped_twist = TwistStamped()
        stamped_twist.header.stamp = self.get_clock().now().to_msg()
        stamped_twist.header.frame_id = ""  # You can leave this empty
        
        stamped_twist.twist.linear.x = x_cmd
        stamped_twist.twist.linear.y = y_cmd
        stamped_twist.twist.linear.z = z_cmd
        stamped_twist.twist.angular.x = 0.0
        stamped_twist.twist.angular.y = 0.0
        stamped_twist.twist.angular.z = th_cmd
        
        self.publisher_.publish(stamped_twist)
    
    def stop_robot(self):
        self.get_logger().info('Stopping robot...')
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.th = 0.0
        self.publish_loop()
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    global settings
    try:
        settings = termios.tcgetattr(sys.stdin)
    except termios.error:
        print("Could not get termios settings. Are you running in a TTY?")
        return

    rclpy.init(args=args)
    
    mecanum_teleop = MecanumTeleop()
    
    try:
        rclpy.spin(mecanum_teleop)  # Spin the node
    except KeyboardInterrupt:
        pass  # CTRL-C is handled in the key_loop thread
    except Exception as e:
        print(f"Spin failed: {e}")
    finally:
        mecanum_teleop.stop_robot()
        mecanum_teleop.destroy_node()
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("Shutdown complete.")

if __name__ == '__main__':
    main()

