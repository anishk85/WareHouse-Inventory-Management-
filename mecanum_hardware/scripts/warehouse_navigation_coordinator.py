#!/usr/bin/env python3
"""
Warehouse Navigation Coordinator
Manages autonomous navigation with lift height control and safety integration

Features:
- Navigates through recorded waypoints sequentially
- Ensures lift reaches target height before moving to next waypoint
- Zigzag pattern: max height → min height → max height → ...
- Integration with optical flow safety system
- Automatic retry on collision detection
- Manual goal acceptance from RViz
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Range
import json
import time
from enum import Enum


class NavigationState(Enum):
    IDLE = 0
    WAITING_FOR_LIFT = 1
    NAVIGATING = 2
    AT_WAYPOINT = 3
    COLLISION_WAIT = 4
    COMPLETED = 5


class WarehouseNavigationCoordinator(Node):
    def __init__(self):
        super().__init__('warehouse_navigation_coordinator')
        
        # Declare parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('min_height', 0.7)  # 70cm minimum
        self.declare_parameter('max_height', 1.6)  # 160cm maximum
        self.declare_parameter('height_tolerance', 0.05)  # ±5cm
        self.declare_parameter('collision_retry_delay', 10.0)  # 10 seconds
        self.declare_parameter('use_manual_goals', False)  # RViz manual mode
        self.declare_parameter('start_with_max_height', True)  # First waypoint pattern
        
        # Get parameters
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.height_tolerance = self.get_parameter('height_tolerance').value
        self.collision_retry_delay = self.get_parameter('collision_retry_delay').value
        self.use_manual_goals = self.get_parameter('use_manual_goals').value
        self.start_with_max_height = self.get_parameter('start_with_max_height').value
        
        # State variables
        self.state = NavigationState.IDLE
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.current_height = None
        self.target_height = None
        self.collision_detected = False
        self.collision_timestamp = None
        self.lift_at_target = False
        
        # Pattern control (zigzag)
        self.current_target_is_max = self.start_with_max_height
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None
        
        # Subscribers
        self.height_sub = self.create_subscription(
            Float32,
            'height_monitor/current_height',
            self.height_callback,
            10
        )
        
        self.collision_sub = self.create_subscription(
            Bool,
            'lift/stop',  # Common topic from optical flow and depth
            self.collision_callback,
            10
        )
        
        self.manual_goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',  # RViz Nav2 goal
            self.manual_goal_callback,
            10
        )
        
        # Publishers
        self.actuator_cmd_pub = self.create_publisher(
            String,
            'actuator/command',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'navigation/status',
            10
        )
        
        # Timer for state machine
        self.state_timer = self.create_timer(0.5, self.state_machine)
        
        # Load waypoints if file provided
        if self.waypoint_file and not self.use_manual_goals:
            self.load_waypoints()
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('WAREHOUSE NAVIGATION COORDINATOR')
        self.get_logger().info('=' * 70)
        if self.use_manual_goals:
            self.get_logger().info('MODE: Manual RViz Goals')
            self.get_logger().info('  Use 2D Nav Goal in RViz to set waypoints')
        else:
            self.get_logger().info('MODE: Autonomous Waypoint Navigation')
            self.get_logger().info(f'  Waypoint file: {self.waypoint_file}')
            self.get_logger().info(f'  Total waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Height range: {self.min_height}m - {self.max_height}m')
        self.get_logger().info(f'Zigzag pattern: Start with {"MAX" if self.start_with_max_height else "MIN"} height')
        self.get_logger().info(f'Collision retry delay: {self.collision_retry_delay}s')
        self.get_logger().info('=' * 70)
    
    def load_waypoints(self):
        """Load waypoints from JSON file"""
        try:
            with open(self.waypoint_file, 'r') as f:
                data = json.load(f)
            
            self.waypoints = data.get('waypoints', [])
            self.get_logger().info(f'✓ Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}')
            
            # Log waypoint summary
            for i, wp in enumerate(self.waypoints):
                self.get_logger().info(
                    f'  Waypoint {i+1}: x={wp["position"]["x"]:.2f}, '
                    f'y={wp["position"]["y"]:.2f}, '
                    f'yaw={wp["yaw_degrees"]:.1f}°'
                )
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            self.waypoints = []
    
    def height_callback(self, msg):
        """Update current height from laser sensor"""
        self.current_height = msg.data
    
    def collision_callback(self, msg):
        """Handle collision detection from optical flow or depth estimation"""
        if msg.data and not self.collision_detected:
            # New collision detected
            self.collision_detected = True
            self.collision_timestamp = time.time()
            self.get_logger().warn('🚨 COLLISION DETECTED - Stopping actuator')
            self.send_actuator_command('stop')
            self.publish_status('collision_detected')
        elif not msg.data and self.collision_detected:
            # Collision cleared
            elapsed = time.time() - self.collision_timestamp if self.collision_timestamp else 0
            if elapsed >= self.collision_retry_delay:
                self.get_logger().info(f'✓ Collision clear after {elapsed:.1f}s - Resuming')
                self.collision_detected = False
                self.collision_timestamp = None
                self.publish_status('collision_cleared')
    
    def manual_goal_callback(self, msg):
        """Handle manual goal from RViz"""
        if not self.use_manual_goals:
            return
        
        self.get_logger().info('📍 Manual goal received from RViz')
        
        # Create waypoint from manual goal
        waypoint = {
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        
        # Add to waypoints and navigate
        self.waypoints = [waypoint]
        self.current_waypoint_idx = 0
        self.state = NavigationState.IDLE
        
        # Trigger state machine
        self.state_machine()
    
    def state_machine(self):
        """Main state machine for navigation coordination"""
        
        if self.state == NavigationState.IDLE:
            # Check if we have waypoints to navigate
            if self.current_waypoint_idx < len(self.waypoints):
                self.get_logger().info('=' * 70)
                self.get_logger().info(
                    f'Starting waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}'
                )
                
                # Determine target height (zigzag pattern)
                self.target_height = self.max_height if self.current_target_is_max else self.min_height
                
                self.get_logger().info(
                    f'Target height: {self.target_height*100:.0f}cm '
                    f'({"MAX" if self.current_target_is_max else "MIN"})'
                )
                
                # Transition to waiting for lift
                self.state = NavigationState.WAITING_FOR_LIFT
                self.lift_at_target = False
                self.move_lift_to_target()
            
            elif self.current_waypoint_idx >= len(self.waypoints) and len(self.waypoints) > 0:
                # All waypoints completed
                if self.state != NavigationState.COMPLETED:
                    self.get_logger().info('=' * 70)
                    self.get_logger().info('✅ ALL WAYPOINTS COMPLETED!')
                    self.get_logger().info('=' * 70)
                    self.state = NavigationState.COMPLETED
                    self.publish_status('all_waypoints_completed')
        
        elif self.state == NavigationState.WAITING_FOR_LIFT:
            # Check if collision detected
            if self.collision_detected:
                self.state = NavigationState.COLLISION_WAIT
                return
            
            # Check if lift reached target height
            if self.current_height is not None:
                height_error = abs(self.current_height - self.target_height)
                
                if height_error <= self.height_tolerance:
                    if not self.lift_at_target:
                        self.lift_at_target = True
                        self.get_logger().info(
                            f'✓ Lift at target height: {self.current_height*100:.1f}cm'
                        )
                        self.send_actuator_command('stop')
                        
                        # Transition to navigation
                        self.state = NavigationState.NAVIGATING
                        self.navigate_to_current_waypoint()
                else:
                    # Log progress periodically
                    if hasattr(self, '_last_height_log'):
                        if time.time() - self._last_height_log > 2.0:
                            self.get_logger().info(
                                f'Moving lift: current={self.current_height*100:.1f}cm, '
                                f'target={self.target_height*100:.1f}cm, '
                                f'error={height_error*100:.1f}cm'
                            )
                            self._last_height_log = time.time()
                    else:
                        self._last_height_log = time.time()
        
        elif self.state == NavigationState.NAVIGATING:
            # Navigation is handled by action client callback
            # This state just monitors for collision
            if self.collision_detected:
                self.cancel_navigation()
                self.state = NavigationState.COLLISION_WAIT
        
        elif self.state == NavigationState.AT_WAYPOINT:
            # Waypoint reached, prepare for next
            self.get_logger().info(f'✓ Waypoint {self.current_waypoint_idx + 1} completed')
            
            # Toggle height target for zigzag pattern
            self.current_target_is_max = not self.current_target_is_max
            
            # Move to next waypoint
            self.current_waypoint_idx += 1
            self.state = NavigationState.IDLE
            self.publish_status(f'waypoint_{self.current_waypoint_idx}_completed')
        
        elif self.state == NavigationState.COLLISION_WAIT:
            # Wait for collision to clear and retry delay
            if not self.collision_detected:
                self.get_logger().info('Resuming from collision wait')
                
                # Go back to waiting for lift (resume height adjustment)
                self.state = NavigationState.WAITING_FOR_LIFT
                self.move_lift_to_target()
    
    def move_lift_to_target(self):
        """Command lift to move to target height"""
        if self.current_height is None:
            self.get_logger().warn('No height data available yet')
            return
        
        height_error = self.target_height - self.current_height
        
        if abs(height_error) <= self.height_tolerance:
            self.get_logger().info('Lift already at target height')
            return
        
        # Determine direction
        if height_error > 0:
            # Need to go up
            self.send_actuator_command('up')
            self.get_logger().info(f'🔼 Moving lift UP to {self.target_height*100:.0f}cm')
        else:
            # Need to go down
            self.send_actuator_command('down')
            self.get_logger().info(f'🔽 Moving lift DOWN to {self.target_height*100:.0f}cm')
    
    def navigate_to_current_waypoint(self):
        """Send navigation goal for current waypoint"""
        if self.current_waypoint_idx >= len(self.waypoints):
            return
        
        waypoint = self.waypoints[self.current_waypoint_idx]
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = waypoint['position']['x']
        goal_msg.pose.pose.position.y = waypoint['position']['y']
        goal_msg.pose.pose.position.z = waypoint['position']['z']
        
        goal_msg.pose.pose.orientation.x = waypoint['orientation']['x']
        goal_msg.pose.pose.orientation.y = waypoint['orientation']['y']
        goal_msg.pose.pose.orientation.z = waypoint['orientation']['z']
        goal_msg.pose.pose.orientation.w = waypoint['orientation']['w']
        
        self.get_logger().info(
            f'🚀 Navigating to waypoint {self.current_waypoint_idx + 1}: '
            f'x={waypoint["position"]["x"]:.2f}, y={waypoint["position"]["y"]:.2f}'
        )
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        self.nav_goal_handle = future.result()
        
        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.state = NavigationState.IDLE
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get result
        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Can log progress here if needed
        pass
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Navigation succeeded')
            self.state = NavigationState.AT_WAYPOINT
            self.publish_status('navigation_succeeded')
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            # Could implement retry logic here
            self.state = NavigationState.IDLE
            self.publish_status('navigation_failed')
    
    def cancel_navigation(self):
        """Cancel current navigation goal"""
        if self.nav_goal_handle is not None:
            self.get_logger().warn('Canceling navigation due to collision')
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
    
    def send_actuator_command(self, command):
        """Send command to actuator"""
        msg = String()
        msg.data = command
        self.actuator_cmd_pub.publish(msg)
    
    def publish_status(self, status):
        """Publish navigation status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = WarehouseNavigationCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt detected')
    except Exception as e:
        if node:
            node.get_logger().error(f'Error: {e}')
        else:
            print(f'Failed to initialize node: {e}')
    finally:
        if node:
            node.get_logger().info('Shutting down Navigation Coordinator')
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()