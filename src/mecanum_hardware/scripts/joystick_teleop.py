#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

class MecanumXboxTeleop(Node):
    def __init__(self):
        super().__init__('mecanum_xbox_teleop')
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)
        
        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Xbox controller button/axis mapping
        self.AXIS_LEFT_X = 0      # Left stick horizontal (strafe left/right)
        self.AXIS_LEFT_Y = 1      # Left stick vertical (forward/backward)
        self.AXIS_RIGHT_X = 3     # Right stick horizontal (rotate)
        self.AXIS_LT = 2          # Left trigger (speed reduction)
        self.AXIS_RT = 5          # Right trigger (speed boost)
        self.BUTTON_A = 0         # Emergency stop
        self.BUTTON_B = 1         # Enable/disable
        
        # State variables
        self.enabled = True
        self.turbo_mode = False
        self.last_joy_msg = None
        self.last_button_b_state = 0
        self.last_button_a_state = 0
        
        # Safety timer - stop if no joy message received
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        self.last_joy_time = self.get_clock().now()
        
        self.get_logger().info('Mecanum Xbox Teleop Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick: Forward/Backward & Strafe Left/Right')
        self.get_logger().info('  Right Stick (horizontal): Rotate')
        self.get_logger().info('  LT: Slow mode (hold)')
        self.get_logger().info('  RT: Turbo mode (hold)')
        self.get_logger().info('  B Button: Toggle Enable/Disable')
        self.get_logger().info('  A Button: Emergency Stop')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick values"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the value to account for deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def joy_callback(self, msg):
        """Process joystick input and publish velocity commands"""
        self.last_joy_msg = msg
        self.last_joy_time = self.get_clock().now()
        
        # Handle button presses with debounce
        current_button_a = msg.buttons[self.BUTTON_A] if len(msg.buttons) > self.BUTTON_A else 0
        current_button_b = msg.buttons[self.BUTTON_B] if len(msg.buttons) > self.BUTTON_B else 0
        
        # Emergency stop (A button) - trigger on rising edge
        if current_button_a and not self.last_button_a_state:
            self.emergency_stop()
            self.last_button_a_state = current_button_a
            return
        self.last_button_a_state = current_button_a
        
        # Toggle enable (B button) - trigger on rising edge
        if current_button_b and not self.last_button_b_state:
            self.enabled = not self.enabled
            status = "ENABLED" if self.enabled else "DISABLED"
            self.get_logger().info(f'Teleop {status}')
        self.last_button_b_state = current_button_b
        
        if not self.enabled:
            return
        
        # Get axis values with deadzone
        left_x = self.apply_deadzone(msg.axes[self.AXIS_LEFT_X]) if len(msg.axes) > self.AXIS_LEFT_X else 0.0
        left_y = self.apply_deadzone(msg.axes[self.AXIS_LEFT_Y]) if len(msg.axes) > self.AXIS_LEFT_Y else 0.0
        right_x = self.apply_deadzone(msg.axes[self.AXIS_RIGHT_X]) if len(msg.axes) > self.AXIS_RIGHT_X else 0.0
        
        # Get trigger values (range: 1.0 to -1.0, where -1.0 is fully pressed)
        lt_value = msg.axes[self.AXIS_LT] if len(msg.axes) > self.AXIS_LT else 1.0
        rt_value = msg.axes[self.AXIS_RT] if len(msg.axes) > self.AXIS_RT else 1.0
        
        # Calculate speed multiplier based on triggers
        # LT pressed (value approaching -1.0) = slow mode (0.3x speed)
        # RT pressed (value approaching -1.0) = turbo mode (1.5x speed)
        # Default = 1.0x speed
        speed_multiplier = 1.0
        
        lt_pressed = (1.0 - lt_value) / 2.0  # Convert to 0.0-1.0 range
        rt_pressed = (1.0 - rt_value) / 2.0  # Convert to 0.0-1.0 range
        
        if lt_pressed > 0.1:
            speed_multiplier = 0.3
        elif rt_pressed > 0.1:
            speed_multiplier = 1.5
        
        # Create Twist message
        twist = Twist()
        
        # Mecanum drive control
        # Linear X: forward/backward (left stick Y)
        # Linear Y: strafe left/right (left stick X)
        # Angular Z: rotate (right stick X)
        
        twist.linear.x = left_y * self.max_linear_speed * self.scale_linear * speed_multiplier
        twist.linear.y = -left_x * self.max_linear_speed * self.scale_linear * speed_multiplier  # Inverted for natural control
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -right_x * self.max_angular_speed * self.scale_angular * speed_multiplier  # Inverted for natural control
        
        # Publish
        self.cmd_vel_pub.publish(twist)
        
        # Log velocity commands when non-zero
        if abs(twist.linear.x) > 0.01 or abs(twist.linear.y) > 0.01 or abs(twist.angular.z) > 0.01:
            speed_mode = "SLOW" if speed_multiplier < 0.5 else ("TURBO" if speed_multiplier > 1.0 else "NORMAL")
            self.get_logger().info(
                f'[{speed_mode}] Publishing -> linear: [x={twist.linear.x:.2f}, y={twist.linear.y:.2f}], angular: [z={twist.angular.z:.2f}]',
                throttle_duration_sec=0.5  # Log at most every 0.5 seconds
            )

    def emergency_stop(self):
        """Send zero velocity command"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        self.enabled = False

    def watchdog_callback(self):
        """Safety watchdog - stop robot if no joy messages received"""
        time_since_last_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        
        if time_since_last_joy > 1.0 and self.last_joy_msg is not None:
            # No joy message for 1 second - send stop command
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            if self.enabled:
                self.get_logger().warn('No joystick input - stopping robot')
                self.enabled = False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MecanumXboxTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before exiting
        if 'node' in locals():
            twist = Twist()
            node.cmd_vel_pub.publish(twist)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()