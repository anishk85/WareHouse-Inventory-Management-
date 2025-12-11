#!/usr/bin/env python3
"""
Height Monitor Node - Enforces safe operating height limits
Monitors laser distance sensor and publishes warnings when limits are reached

Safety Limits:
- Minimum Height: 0.7m (70cm)
- Maximum Height: 1.6m (160cm)

Topics:
- Subscribes: /laser_distance (sensor_msgs/Range)
- Publishes: 
    - /height_monitor/status (String) - Current status
    - /height_monitor/current_height (Float32) - Current height in meters
    - /height_monitor/warning (String) - Warning messages when limits reached
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Range
import time


class HeightMonitorNode(Node):
    def __init__(self):
        super().__init__('height_monitor_node')
        
        # Declare parameters for height limits
        self.declare_parameter('min_height', 0.7)  # 70cm minimum
        self.declare_parameter('max_height', 1.6)  # 160cm maximum
        self.declare_parameter('warning_margin', 0.05)  # 5cm warning zone
        
        # Get parameters
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.warning_margin = self.get_parameter('warning_margin').value
        
        # Calculate warning zones
        self.min_warning_threshold = self.min_height + self.warning_margin
        self.max_warning_threshold = self.max_height - self.warning_margin
        
        # Subscribe to laser distance sensor
        self.laser_sub = self.create_subscription(
            Range,
            'laser_distance',
            self.laser_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'height_monitor/status', 10)
        self.height_pub = self.create_publisher(Float32, 'height_monitor/current_height', 10)
        self.warning_pub = self.create_publisher(String, 'height_monitor/warning', 10)
        
        # State tracking
        self.current_height = None
        self.last_warning_time = 0
        self.warning_cooldown = 1.0  # seconds between repeated warnings
        self.last_status = 'OK'
        
        # Statistics
        self.measurement_count = 0
        self.warning_count = 0
        self.violation_count = 0
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('HEIGHT MONITOR NODE INITIALIZED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Minimum Height: {self.min_height*100:.1f} cm')
        self.get_logger().info(f'Maximum Height: {self.max_height*100:.1f} cm')
        self.get_logger().info(f'Warning Margin: {self.warning_margin*100:.1f} cm')
        self.get_logger().info(f'Warning Zone:   {self.min_warning_threshold*100:.1f} cm - '
                             f'{self.max_warning_threshold*100:.1f} cm')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Subscribed to: /laser_distance')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /height_monitor/status')
        self.get_logger().info('  - /height_monitor/current_height')
        self.get_logger().info('  - /height_monitor/warning')
        self.get_logger().info('=' * 70)
    
    def laser_callback(self, msg):
        """Process laser distance readings and enforce height limits"""
        self.measurement_count += 1
        distance = msg.range
        
        # Publish current height
        self.current_height = distance
        height_msg = Float32()
        height_msg.data = distance
        self.height_pub.publish(height_msg)
        
        # Determine status
        current_time = time.time()
        status = self.check_height_status(distance)
        
        # Publish status
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
        # Handle warnings and violations
        if status != 'OK' and status != self.last_status:
            # Status changed, publish warning
            self.publish_warning(status, distance)
            self.last_status = status
        elif status != 'OK' and (current_time - self.last_warning_time) > self.warning_cooldown:
            # Repeated warning (with cooldown)
            self.publish_warning(status, distance)
        
        # Log periodically
        if self.measurement_count % 50 == 0:
            self.log_status(distance, status)
    
    def check_height_status(self, distance):
        """
        Check height and return status string
        Returns: 'OK', 'WARNING_MIN', 'WARNING_MAX', 'VIOLATION_MIN', 'VIOLATION_MAX'
        """
        if distance < self.min_height:
            self.violation_count += 1
            return 'VIOLATION_MIN'
        elif distance > self.max_height:
            self.violation_count += 1
            return 'VIOLATION_MAX'
        elif distance < self.min_warning_threshold:
            return 'WARNING_MIN'
        elif distance > self.max_warning_threshold:
            return 'WARNING_MAX'
        else:
            return 'OK'
    
    def publish_warning(self, status, distance):
        """Publish warning message"""
        self.warning_count += 1
        self.last_warning_time = time.time()
        
        warning_msg = String()
        
        if status == 'VIOLATION_MIN':
            warning_msg.data = f'MINIMUM HEIGHT REACHED! Current: {distance*100:.1f}cm, Limit: {self.min_height*100:.1f}cm'
            self.get_logger().error(f'🚨 {warning_msg.data}')
        elif status == 'VIOLATION_MAX':
            warning_msg.data = f'MAXIMUM HEIGHT REACHED! Current: {distance*100:.1f}cm, Limit: {self.max_height*100:.1f}cm'
            self.get_logger().error(f'🚨 {warning_msg.data}')
        elif status == 'WARNING_MIN':
            warning_msg.data = f'WARNING: Approaching minimum height! Current: {distance*100:.1f}cm, Limit: {self.min_height*100:.1f}cm'
            self.get_logger().warn(f'⚠️  {warning_msg.data}')
        elif status == 'WARNING_MAX':
            warning_msg.data = f'WARNING: Approaching maximum height! Current: {distance*100:.1f}cm, Limit: {self.max_height*100:.1f}cm'
            self.get_logger().warn(f'⚠️  {warning_msg.data}')
        
        self.warning_pub.publish(warning_msg)
    
    def log_status(self, distance, status):
        """Log current status"""
        distance_cm = distance * 100
        min_cm = self.min_height * 100
        max_cm = self.max_height * 100
        
        # Create visual bar
        bar = self.create_height_bar(distance)
        
        self.get_logger().info('─' * 70)
        self.get_logger().info(f'Height: {distance_cm:6.1f}cm | Status: {status:15s} | '
                             f'Measurements: {self.measurement_count} | Warnings: {self.warning_count}')
        self.get_logger().info(f'Range:  {min_cm:6.1f}cm ←→ {max_cm:6.1f}cm')
        self.get_logger().info(bar)
        self.get_logger().info('─' * 70)
    
    def create_height_bar(self, distance):
        """Create visual representation of height"""
        bar_width = 50
        
        # Normalize distance to bar position
        range_span = self.max_height - self.min_height
        if range_span > 0:
            normalized = (distance - self.min_height) / range_span
            pos = int(normalized * bar_width)
            pos = max(0, min(bar_width - 1, pos))
        else:
            pos = bar_width // 2
        
        # Build bar
        bar = [' '] * bar_width
        
        # Mark limits
        bar[0] = '['
        bar[-1] = ']'
        
        # Mark warning zones
        warning_min_pos = int((self.min_warning_threshold - self.min_height) / range_span * bar_width)
        warning_max_pos = int((self.max_warning_threshold - self.min_height) / range_span * bar_width)
        
        for i in range(1, warning_min_pos):
            bar[i] = '░'
        for i in range(warning_max_pos + 1, bar_width - 1):
            bar[i] = '░'
        
        # Mark current position
        if distance < self.min_height:
            bar[0] = '◄'  # Below minimum
        elif distance > self.max_height:
            bar[-1] = '►'  # Above maximum
        else:
            bar[pos] = '●'
        
        return ''.join(bar) + f'  ({distance*100:.1f}cm)'


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = HeightMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('\nKeyboard interrupt detected')
    except Exception as e:
        if node:
            node.get_logger().error(f'Error: {e}')
        else:
            print(f'Failed to initialize node: {e}')
    finally:
        if node:
            node.get_logger().info('Shutting down Height Monitor Node')
            node.get_logger().info(f'Total measurements: {node.measurement_count}')
            node.get_logger().info(f'Total warnings: {node.warning_count}')
            node.get_logger().info(f'Total violations: {node.violation_count}')
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()