#!/usr/bin/env python3
"""
Enhanced Joystick Teleop with Waypoint Saving

This node allows:
1. Normal teleop control with joystick
2. Press a button (e.g., 'A' button) to save current pose as waypoint
3. Saves waypoints to a YAML file for later use in autonomous navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
import yaml
import os
from datetime import datetime


class EnhancedJoystickTeleopNode(Node):
    """
    Enhanced joystick teleop with waypoint recording capability
    """
    
    def __init__(self):
        super().__init__('enhanced_joystick_teleop')
        
        # Parameters
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('waypoints_file', '/tmp/waypoints.yaml')
        self.declare_parameter('save_button', 0)  # A button on Xbox controller
        self.declare_parameter('use_odom', False)  # Use odometry instead of TF
        
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.save_button = self.get_parameter('save_button').value
        self.use_odom = self.get_parameter('use_odom').value
        
        # Waypoints storage
        self.waypoints = []
        self.last_button_state = 0
        
        # Current pose
        self.current_pose = None
        
        # TF2 Buffer and Listener (for getting robot pose from map)
        if not self.use_odom:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        if self.use_odom:
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
        
        # Timer for getting TF
        if not self.use_odom:
            self.tf_timer = self.create_timer(0.1, self.update_pose_from_tf)
        
        # Load existing waypoints if file exists
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        self.waypoints = data['waypoints']
                        self.get_logger().info(
                            f'Loaded {len(self.waypoints)} existing waypoints'
                        )
            except Exception as e:
                self.get_logger().warn(f'Could not load existing waypoints: {e}')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Enhanced Joystick Teleop with Waypoint Saving')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Waypoints file: {self.waypoints_file}')
        self.get_logger().info(f'Save button: Button {self.save_button}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('CONTROLS:')
        self.get_logger().info('  Left Stick: Move forward/backward')
        self.get_logger().info('  Right Stick: Rotate')
        self.get_logger().info(f'  Button {self.save_button}: Save current position as waypoint')
        self.get_logger().info('=' * 60)
    
    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
    
    def update_pose_from_tf(self):
        """Update current pose from TF (map -> base_link)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Convert transform to pose
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            self.current_pose = pose
            
        except Exception as e:
            # TF not available yet (map might not exist)
            pass
    
    def joy_callback(self, msg: Joy):
        """Handle joystick input"""
        # Teleop control
        twist = Twist()
        
        # Typical joystick layout (Xbox controller):
        # Axes: [LX, LY, LT, RX, RY, RT, DX, DY]
        # Buttons: [A, B, X, Y, LB, RB, Back, Start, ...]
        
        if len(msg.axes) > 1:
            twist.linear.x = msg.axes[1] * self.linear_scale  # Left stick Y
        if len(msg.axes) > 3:
            twist.angular.z = msg.axes[3] * self.angular_scale  # Right stick X
        
        self.cmd_vel_pub.publish(twist)
        
        # Waypoint saving (detect button press edge)
        if len(msg.buttons) > self.save_button:
            button_state = msg.buttons[self.save_button]
            
            # Button pressed (rising edge)
            if button_state == 1 and self.last_button_state == 0:
                self.save_current_waypoint()
            
            self.last_button_state = button_state
    
    def save_current_waypoint(self):
        """Save current robot pose as a waypoint"""
        if self.current_pose is None:
            self.get_logger().warn('⚠️  Cannot save waypoint: Pose not available yet')
            self.get_logger().warn('    Make sure robot is localized (map frame exists)')
            return
        
        # Create waypoint dictionary
        waypoint = {
            'name': f'waypoint_{len(self.waypoints) + 1}',
            'position': {
                'x': float(self.current_pose.position.x),
                'y': float(self.current_pose.position.y),
                'z': float(self.current_pose.position.z)
            },
            'orientation': {
                'x': float(self.current_pose.orientation.x),
                'y': float(self.current_pose.orientation.y),
                'z': float(self.current_pose.orientation.z),
                'w': float(self.current_pose.orientation.w)
            },
            'timestamp': datetime.now().isoformat()
        }
        
        self.waypoints.append(waypoint)
        
        # Save to file
        try:
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            
            data = {
                'waypoints': self.waypoints,
                'metadata': {
                    'created': datetime.now().isoformat(),
                    'frame_id': 'map',
                    'total_waypoints': len(self.waypoints)
                }
            }
            
            with open(self.waypoints_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'✓ WAYPOINT {len(self.waypoints)} SAVED!')
            self.get_logger().info(f'  Position: ({waypoint["position"]["x"]:.2f}, '
                                   f'{waypoint["position"]["y"]:.2f})')
            self.get_logger().info(f'  File: {self.waypoints_file}')
            self.get_logger().info(f'  Total waypoints: {len(self.waypoints)}')
            self.get_logger().info('=' * 60)
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to save waypoint: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = EnhancedJoystickTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt')
    except Exception as e:
        if node:
            node.get_logger().error(f'Error: {e}')
        else:
            print(f'Failed to initialize node: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
