#!/usr/bin/env python3
"""
Waypoint Follower Node with Inventory Task Execution

This node orchestrates autonomous inventory scanning:

WORKFLOW:
1. START: Enable QR detection (runs continuously throughout mission)
2. NAVIGATE: Move to first waypoint using Nav2
3. AT WAYPOINT: Execute inventory task:
   a. Start lifting actuator (QR detection captures codes during lift)
   b. Wait at top for full shelf scan (configurable duration, default 50s)
   c. Lower actuator back down
4. REPEAT: Navigate to next waypoint
5. COMPLETE: All waypoints done (QR detection keeps running until shutdown)

KEY FEATURE: QR detection starts ONCE at mission start and runs continuously.
This allows capturing QR codes while the actuator lifts through different shelf levels.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from warehouse_rover_msgs.msg import QRDetectionArray
import yaml
import time
from enum import Enum


class TaskState(Enum):
    IDLE = 0
    STARTING_QR_DETECTION = 1
    NAVIGATING = 2
    REACHED_WAYPOINT = 3
    LIFTING_ACTUATOR = 4
    WAITING_SCAN_COMPLETE = 5
    LOWERING_ACTUATOR = 6
    COMPLETED = 7


class WaypointFollowerNode(Node):
    """
    Autonomous waypoint navigation with inventory tasks
    """
    
    def __init__(self):
        super().__init__('waypoint_follower_node')
        
        # Parameters
        self.declare_parameter('waypoints_file', '/tmp/waypoints.yaml')
        self.declare_parameter('scan_duration', 50.0)  # 50 seconds for scanning
        self.declare_parameter('actuator_lift_time', 5.0)  # Time to lift up
        self.declare_parameter('actuator_lower_time', 5.0)  # Time to lower
        self.declare_parameter('use_initial_pose', True)  # Whether to navigate to initial pose first
        
        waypoints_file = self.get_parameter('waypoints_file').value
        self.scan_duration = self.get_parameter('scan_duration').value
        self.lift_time = self.get_parameter('actuator_lift_time').value
        self.lower_time = self.get_parameter('actuator_lower_time').value
        self.use_initial_pose = self.get_parameter('use_initial_pose').value
        
        # State
        self.state = TaskState.IDLE
        self.waypoints = []
        self.current_waypoint_idx = -1
        self.qr_detections_count = 0
        self.scan_start_time = None
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.actuator_cmd_pub = self.create_publisher(String, '/actuator/command', 10)
        self.status_pub = self.create_publisher(String, '/waypoint_follower/status', 10)
        self.qr_enable_pub = self.create_publisher(Bool, '/qr_enable', 10)
        
        # Subscribers
        self.qr_sub = self.create_subscription(
            QRDetectionArray,
            '/qr_detections',
            self.qr_callback,
            10
        )
        
        # QR Detection state
        self.qr_detection_started = False
        
        # Timer for state machine
        self.timer = self.create_timer(0.5, self.state_machine)
        
        # Load waypoints
        try:
            self.load_waypoints(waypoints_file)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            self.waypoints = []
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Waypoint Follower Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Scan Duration: {self.scan_duration}s')
        self.get_logger().info('=' * 60)
    
    def load_waypoints(self, filepath: str):
        """Load waypoints from YAML file"""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        
        if 'waypoints' in data:
            self.waypoints = data['waypoints']
        else:
            self.waypoints = []
    
    def qr_callback(self, msg: QRDetectionArray):
        """Count QR detections during scanning"""
        if self.state == TaskState.WAITING_SCAN_COMPLETE and msg.detection_count > 0:
            self.qr_detections_count += msg.detection_count
            self.get_logger().info(
                f'QR Detections: {self.qr_detections_count} '
                f'(+{msg.detection_count} new)'
            )
    
    def enable_qr_detection(self, enable: bool):
        """Enable or disable QR detection"""
        msg = Bool()
        msg.data = enable
        self.qr_enable_pub.publish(msg)
        status = "ENABLED" if enable else "DISABLED"
        self.get_logger().info(f'QR Detection {status}')
    
    def send_nav_goal(self, pose_dict: dict):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = pose_dict['position']['x']
        goal_msg.pose.pose.position.y = pose_dict['position']['y']
        goal_msg.pose.pose.position.z = pose_dict['position'].get('z', 0.0)
        
        goal_msg.pose.pose.orientation.x = pose_dict['orientation']['x']
        goal_msg.pose.pose.orientation.y = pose_dict['orientation']['y']
        goal_msg.pose.pose.orientation.z = pose_dict['orientation']['z']
        goal_msg.pose.pose.orientation.w = pose_dict['orientation']['w']
        
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
            f'({pose_dict["position"]["x"]:.2f}, {pose_dict["position"]["y"]:.2f})'
        )
        
        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Could display distance to goal, ETA, etc.
        pass
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.state = TaskState.IDLE
            return
        
        self.get_logger().info('Navigation goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        
        if result:
            self.get_logger().info('✓ Waypoint reached!')
            self.state = TaskState.REACHED_WAYPOINT
            self.publish_status(f'Reached waypoint {self.current_waypoint_idx + 1}')
        else:
            self.get_logger().error('Navigation failed!')
            self.state = TaskState.IDLE
    
    def send_actuator_command(self, command: str):
        """Send command to actuator"""
        msg = String()
        msg.data = command
        self.actuator_cmd_pub.publish(msg)
        self.get_logger().info(f'Actuator command: {command}')
    
    def publish_status(self, status: str):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def state_machine(self):
        """Main state machine for waypoint following and task execution"""
        
        if self.state == TaskState.IDLE:
            # Start mission if waypoints are loaded
            if self.waypoints and self.current_waypoint_idx == -1:
                self.get_logger().info('=' * 60)
                self.get_logger().info('  STARTING INVENTORY MISSION')
                self.get_logger().info('=' * 60)
                self.state = TaskState.STARTING_QR_DETECTION
        
        elif self.state == TaskState.STARTING_QR_DETECTION:
            # STEP 1: Enable QR detection FIRST (before any navigation)
            if not self.qr_detection_started:
                self.get_logger().info('[STEP 1/6] Starting QR detection system...')
                self.enable_qr_detection(True)
                self.qr_detection_started = True
                time.sleep(2)  # Give QR system time to initialize
                
                self.get_logger().info('✓ QR detection running continuously')
                self.get_logger().info('[STEP 2/6] Beginning navigation...')
                
                # Now start navigation to first waypoint
                self.current_waypoint_idx = 0
                self.state = TaskState.NAVIGATING
                self.send_nav_goal(self.waypoints[self.current_waypoint_idx])
        
        elif self.state == TaskState.NAVIGATING:
            # Waiting for navigation to complete
            # Result callback will change state
            pass
        
        elif self.state == TaskState.REACHED_WAYPOINT:
            # STEP 3: Waypoint reached, start inventory task
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'  AT WAYPOINT {self.current_waypoint_idx + 1}/{len(self.waypoints)}')
            self.get_logger().info('=' * 60)
            self.get_logger().info('[STEP 3/6] Starting actuator lift...')
            self.get_logger().info('   (QR detection already running in background)')
            
            self.state = TaskState.LIFTING_ACTUATOR
            self.send_actuator_command('up')
            self.publish_status(f'Lifting at waypoint {self.current_waypoint_idx + 1}')
            self.task_start_time = time.time()
            self.qr_detections_count = 0  # Reset counter for this waypoint
        
        elif self.state == TaskState.LIFTING_ACTUATOR:
            # STEP 4: Wait for actuator to lift (QR detection captures during lift)
            elapsed = time.time() - self.task_start_time
            if elapsed >= self.lift_time:
                self.get_logger().info('✓ Actuator fully raised')
                self.get_logger().info('[STEP 4/6] Scanning all shelves...')
                self.get_logger().info(f'   Will scan for {self.scan_duration} seconds')
                self.get_logger().info('   (Camera is capturing QR codes continuously)')
                
                self.state = TaskState.WAITING_SCAN_COMPLETE
                self.scan_start_time = time.time()
                self.publish_status(f'Scanning ({self.scan_duration}s)')
        
        elif self.state == TaskState.WAITING_SCAN_COMPLETE:
            # STEP 5: Wait for scan duration (QR detection keeps running)
            elapsed = time.time() - self.scan_start_time
            remaining = self.scan_duration - elapsed
            
            if elapsed >= self.scan_duration:
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'✓ Scan complete! Detected {self.qr_detections_count} QR codes')
                self.get_logger().info('=' * 60)
                self.get_logger().info('[STEP 5/6] Lowering actuator...')
                
                self.state = TaskState.LOWERING_ACTUATOR
                self.send_actuator_command('down')
                self.publish_status('Lowering actuator')
                self.task_start_time = time.time()
            
            elif int(elapsed) % 10 == 0 and int(elapsed) > 0:  # Log every 10 seconds
                self.get_logger().info(
                    f'   Scanning... {remaining:.0f}s remaining '
                    f'({self.qr_detections_count} detections so far)'
                )
        
        elif self.state == TaskState.LOWERING_ACTUATOR:
            # STEP 6: Wait for actuator to lower (QR detection STILL running)
            elapsed = time.time() - self.task_start_time
            if elapsed >= self.lower_time:
                self.get_logger().info('✓ Actuator lowered')
                self.send_actuator_command('stop')
                
                # Move to next waypoint
                self.current_waypoint_idx += 1
                
                if self.current_waypoint_idx >= len(self.waypoints):
                    # Mission complete
                    self.state = TaskState.COMPLETED
                    self.get_logger().info('')
                    self.get_logger().info('=' * 60)
                    self.get_logger().info('  ALL WAYPOINTS COMPLETED!')
                    self.get_logger().info('=' * 60)
                    self.get_logger().info('  QR detection will keep running')
                    self.get_logger().info('  Press Ctrl+C to stop the mission')
                    self.get_logger().info('=' * 60)
                    self.publish_status('Mission completed')
                else:
                    # Navigate to next waypoint (QR detection keeps running)
                    self.get_logger().info('')
                    self.get_logger().info(f'[STEP 2/6] Moving to next waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}...')
                    self.state = TaskState.NAVIGATING
                    self.send_nav_goal(self.waypoints[self.current_waypoint_idx])
        
        elif self.state == TaskState.COMPLETED:
            # Mission complete, QR detection still running
            # Just idle here, user can Ctrl+C to exit
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = WaypointFollowerNode()
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
