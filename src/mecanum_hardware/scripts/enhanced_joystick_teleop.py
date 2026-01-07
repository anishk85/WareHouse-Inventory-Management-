#!/usr/bin/env python3
"""
Enhanced Joystick Teleop with Waypoint Saving
Adds button to save current position as waypoint for navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from warehouse_rover_msgs.srv import SaveWaypoint
import time


class EnhancedJoystickTeleop(Node):
    """
    Joystick control with waypoint saving capability
    
    Controls:
        Left Stick X: Rotation
        Left Stick Y: Forward/Backward
        Right Stick X: Strafe Left/Right
        Button A (0): Save waypoint as rack
        Button B (1): Save waypoint as station
        Button X (2): Save waypoint as checkpoint
        Button Y (3): Emergency stop
    """
    
    def __init__(self):
        super().__init__('enhanced_joystick_teleop')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('deadzone', 0.1)
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # State
        self.waypoint_counter = {'rack': 1, 'station': 1, 'checkpoint': 1}
        self.last_button_time = {}
        self.button_cooldown = 1.0  # 1 second between button presses
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Service client for saving waypoints
        self.save_waypoint_client = self.create_client(
            SaveWaypoint,
            '/waypoint_manager/save_waypoint'
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Enhanced Joystick Teleop Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick Y:  Forward/Backward')
        self.get_logger().info('  Left Stick X:  Rotation')
        self.get_logger().info('  Right Stick X: Strafe Left/Right')
        self.get_logger().info('')
        self.get_logger().info('Waypoint Buttons:')
        self.get_logger().info('  Button A: Save RACK waypoint')
        self.get_logger().info('  Button B: Save STATION waypoint')
        self.get_logger().info('  Button X: Save CHECKPOINT waypoint')
        self.get_logger().info('  Button Y: Emergency STOP')
        self.get_logger().info('=' * 60)
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def joy_callback(self, msg: Joy):
        """Process joystick input"""
        # Extract axes (may vary by controller)
        # Xbox/PS4 typical layout:
        # axes[0] = Left stick X (left/right)
        # axes[1] = Left stick Y (up/down)
        # axes[2] = Right stick X (left/right)
        # axes[3] = Right stick Y (up/down)
        
        if len(msg.axes) < 4:
            return
        
        # Movement commands
        linear_x = self.apply_deadzone(msg.axes[1]) * self.max_linear  # Forward/back
        linear_y = self.apply_deadzone(msg.axes[2]) * self.max_linear  # Strafe
        angular_z = self.apply_deadzone(msg.axes[0]) * self.max_angular  # Rotation
        
        # Publish velocity
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        # Button handling
        if len(msg.buttons) < 4:
            return
        
        current_time = time.time()
        
        # Button A (0): Save RACK waypoint
        if msg.buttons[0] == 1:
            if self._check_button_cooldown('A', current_time):
                self.save_waypoint('rack')
        
        # Button B (1): Save STATION waypoint
        if msg.buttons[1] == 1:
            if self._check_button_cooldown('B', current_time):
                self.save_waypoint('station')
        
        # Button X (2): Save CHECKPOINT waypoint
        if msg.buttons[2] == 1:
            if self._check_button_cooldown('X', current_time):
                self.save_waypoint('checkpoint')
        
        # Button Y (3): Emergency stop
        if msg.buttons[3] == 1:
            if self._check_button_cooldown('Y', current_time):
                self.emergency_stop()
    
    def _check_button_cooldown(self, button_name, current_time):
        """Check if enough time has passed since last button press"""
        if button_name in self.last_button_time:
            if current_time - self.last_button_time[button_name] < self.button_cooldown:
                return False
        self.last_button_time[button_name] = current_time
        return True
    
    def save_waypoint(self, waypoint_type):
        """Save current position as a waypoint"""
        if not self.save_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waypoint service not available')
            return
        
        # Generate waypoint name
        counter = self.waypoint_counter[waypoint_type]
        waypoint_name = f"{waypoint_type.upper()}_{counter:03d}"
        self.waypoint_counter[waypoint_type] += 1
        
        # Call service
        request = SaveWaypoint.Request()
        request.waypoint_name = waypoint_name
        request.waypoint_type = waypoint_type
        
        future = self.save_waypoint_client.call_async(request)
        future.add_done_callback(
            lambda f: self._waypoint_saved_callback(f, waypoint_name, waypoint_type)
        )
    
    def _waypoint_saved_callback(self, future, waypoint_name, waypoint_type):
        """Callback for waypoint save service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"✓ Saved {waypoint_type.upper()}: '{waypoint_name}' "
                    f"at ({response.x:.2f}, {response.y:.2f})"
                )
            else:
                self.get_logger().error(f"Failed to save waypoint: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def emergency_stop(self):
        """Send stop command"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn('⚠️  EMERGENCY STOP')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = EnhancedJoystickTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
