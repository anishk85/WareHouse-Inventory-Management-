#!/usr/bin/env python3
"""
Nav2 Waypoint Follower with Inventory Task Execution

Uses Nav2's FollowWaypoints action server for robust waypoint navigation.
Executes inventory tasks (actuator control + QR scanning) between waypoints.

KEY FEATURES:
- Uses Nav2's built-in waypoint follower (more reliable)
- Subscribes to waypoint task executor for stop events
- QR detection starts once and runs continuously
- Actuator lifts at each waypoint for shelf scanning
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from warehouse_rover_msgs.msg import QRDetectionArray
import yaml
import time
from enum import Enum
import math


class MissionState(Enum):
    IDLE = 0
    STARTING_QR = 1
    SENDING_WAYPOINTS = 2
    NAVIGATING = 3
    MISSION_COMPLETE = 4
    ERROR = 5


class Nav2WaypointFollower(Node):
    """
    Autonomous waypoint navigation using Nav2's FollowWaypoints action
    
    This node uses Nav2's waypoint_follower server which handles:
    - Sequential navigation through multiple waypoints
    - Automatic recovery from navigation failures
    - Feedback on current waypoint progress
    
    Our node adds:
    - Inventory task execution at each waypoint (via waypoint task executor plugin)
    - QR detection coordination
    - Mission monitoring and logging
    """
    
    def __init__(self):
        super().__init__('nav2_waypoint_follower')
        
        # Parameters
        self.declare_parameter('waypoints_file', '/tmp/waypoints.yaml')
        self.declare_parameter('scan_duration', 50.0)
        self.declare_parameter('actuator_lift_time', 5.0)
        self.declare_parameter('actuator_lower_time', 5.0)
        
        waypoints_file = self.get_parameter('waypoints_file').value
        self.scan_duration = self.get_parameter('scan_duration').value
        self.lift_time = self.get_parameter('actuator_lift_time').value
        self.lower_time = self.get_parameter('actuator_lower_time').value
        
        # State
        self.state = MissionState.IDLE
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.total_qr_detections = 0
        self.qr_detection_started = False
        
        # Nav2 FollowWaypoints Action Client
        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )
        
        # Publishers
        self.actuator_cmd_pub = self.create_publisher(String, '/actuator/command', 10)
        self.qr_enable_pub = self.create_publisher(Bool, '/qr_enable', 10)
        self.status_pub = self.create_publisher(String, '/inventory_mission/status', 10)
        
        # Subscribers
        self.qr_sub = self.create_subscription(
            QRDetectionArray,
            '/qr_detections',
            self.qr_callback,
            10
        )
        
        # Subscribe to waypoint task executor events
        # Nav2's waypoint_follower can trigger tasks at each waypoint
        self.waypoint_task_sub = self.create_subscription(
            String,
            '/waypoint_task_executor/event',
            self.waypoint_task_callback,
            10
        )
        
        # Timer for mission orchestration
        self.timer = self.create_timer(1.0, self.mission_orchestrator)
        
        # Load waypoints
        try:
            self.load_waypoints(waypoints_file)
            self.get_logger().info(f'✓ Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load waypoints: {e}')
            self.state = MissionState.ERROR
            return
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('  NAV2 WAYPOINT FOLLOWER - INVENTORY MISSION')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Waypoints:      {len(self.waypoints)}')
        self.get_logger().info(f'Scan Duration:  {self.scan_duration}s per waypoint')
        self.get_logger().info(f'Lift Time:      {self.lift_time}s')
        self.get_logger().info(f'Lower Time:     {self.lower_time}s')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('Waiting for Nav2 waypoint_follower server...')
    
    def load_waypoints(self, filepath: str):
        """Load waypoints from YAML file"""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        
        if 'waypoints' in data:
            self.waypoints = data['waypoints']
        else:
            raise ValueError('No waypoints found in file')
    
    def qr_callback(self, msg: QRDetectionArray):
        """Count total QR detections throughout mission"""
        if msg.detection_count > 0:
            self.total_qr_detections += msg.detection_count
            if self.total_qr_detections % 10 == 0:  # Log every 10 detections
                self.get_logger().info(
                    f'📊 Total QR detections: {self.total_qr_detections}'
                )
    
    def waypoint_task_callback(self, msg: String):
        """
        Handle waypoint task events from Nav2's waypoint task executor
        
        The waypoint_follower can execute tasks at each waypoint.
        We listen for 'arrived' events to trigger inventory tasks.
        """
        event = msg.data
        
        if 'arrived' in event.lower() or 'reached' in event.lower():
            waypoint_num = self.current_waypoint_idx + 1
            self.get_logger().info('')
            self.get_logger().info('=' * 70)
            self.get_logger().info(f'  📍 ARRIVED AT WAYPOINT {waypoint_num}/{len(self.waypoints)}')
            self.get_logger().info('=' * 70)
            
            # Execute inventory task
            self.execute_inventory_task(waypoint_num)
            
            self.current_waypoint_idx += 1
    
    def execute_inventory_task(self, waypoint_num: int):
        """
        Execute inventory scanning task at current waypoint
        
        Sequence:
        1. Lift actuator (QR detection captures during lift)
        2. Wait at top for full scan
        3. Lower actuator
        
        Note: QR detection is already running continuously
        """
        task_start = time.time()
        
        # STEP 1: Lift actuator
        self.get_logger().info(f'[Task 1/3] Lifting actuator... ({self.lift_time}s)')
        self.send_actuator_command('up')
        self.publish_status(f'Lifting at waypoint {waypoint_num}')
        time.sleep(self.lift_time)
        
        # STEP 2: Scan at top
        self.get_logger().info(f'[Task 2/3] Scanning shelves... ({self.scan_duration}s)')
        self.get_logger().info('   📷 Camera capturing QR codes')
        self.get_logger().info('   💾 Database updating automatically')
        self.publish_status(f'Scanning at waypoint {waypoint_num}')
        
        # Log progress every 10 seconds
        scan_start = time.time()
        while time.time() - scan_start < self.scan_duration:
            elapsed = time.time() - scan_start
            remaining = self.scan_duration - elapsed
            
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                self.get_logger().info(
                    f'   ⏱️  Scanning... {remaining:.0f}s remaining '
                    f'(Total detections: {self.total_qr_detections})'
                )
            time.sleep(0.5)
        
        # STEP 3: Lower actuator
        self.get_logger().info(f'[Task 3/3] Lowering actuator... ({self.lower_time}s)')
        self.send_actuator_command('down')
        self.publish_status(f'Lowering at waypoint {waypoint_num}')
        time.sleep(self.lower_time)
        
        self.send_actuator_command('stop')
        
        task_duration = time.time() - task_start
        self.get_logger().info(f'✓ Waypoint {waypoint_num} complete ({task_duration:.1f}s)')
        self.get_logger().info('')
    
    def send_follow_waypoints_goal(self):
        """
        Send all waypoints to Nav2's FollowWaypoints action server
        
        This is simpler than NavigateToPose - send all waypoints at once,
        Nav2 handles the sequential navigation automatically.
        """
        goal_msg = FollowWaypoints.Goal()
        
        # Convert our waypoints to PoseStamped messages
        for i, wp in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = wp['position']['x']
            pose.pose.position.y = wp['position']['y']
            pose.pose.position.z = wp['position'].get('z', 0.0)
            
            pose.pose.orientation.x = wp['orientation']['x']
            pose.pose.orientation.y = wp['orientation']['y']
            pose.pose.orientation.z = wp['orientation']['z']
            pose.pose.orientation.w = wp['orientation']['w']
            
            goal_msg.poses.append(pose)
        
        self.get_logger().info(f'📤 Sending {len(goal_msg.poses)} waypoints to Nav2...')
        
        # Send goal
        self.send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from Nav2's waypoint follower"""
        feedback = feedback_msg.feedback
        current_wp = feedback.current_waypoint
        
        if current_wp != self.current_waypoint_idx:
            self.current_waypoint_idx = current_wp
            self.get_logger().info(
                f'🚗 Navigating to waypoint {current_wp + 1}/{len(self.waypoints)}'
            )
    
    def goal_response_callback(self, future):
        """Handle goal acceptance from Nav2"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 rejected waypoint following goal!')
            self.state = MissionState.ERROR
            return
        
        self.get_logger().info('✓ Nav2 accepted waypoints, starting navigation...')
        self.state = MissionState.NAVIGATING
        
        # Get result
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle final result from Nav2 waypoint follower"""
        result = future.result().result
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('  🎉 MISSION COMPLETE!')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Total Waypoints:    {len(self.waypoints)}')
        self.get_logger().info(f'Missed Waypoints:   {len(result.missed_waypoints)}')
        self.get_logger().info(f'Total QR Detections: {self.total_qr_detections}')
        self.get_logger().info('=' * 70)
        self.get_logger().info('📷 QR detection still running (Press Ctrl+C to stop)')
        self.get_logger().info('=' * 70)
        
        if result.missed_waypoints:
            self.get_logger().warn(f'⚠️  Missed waypoints: {result.missed_waypoints}')
        
        self.state = MissionState.MISSION_COMPLETE
        self.publish_status('Mission complete')
    
    def send_actuator_command(self, command: str):
        """Send command to actuator"""
        msg = String()
        msg.data = command
        self.actuator_cmd_pub.publish(msg)
    
    def enable_qr_detection(self, enable: bool):
        """Enable or disable QR detection"""
        msg = Bool()
        msg.data = enable
        self.qr_enable_pub.publish(msg)
        status = "ENABLED" if enable else "DISABLED"
        self.get_logger().info(f'📷 QR Detection {status}')
    
    def publish_status(self, status: str):
        """Publish mission status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def mission_orchestrator(self):
        """
        Main mission state machine
        
        Handles high-level mission flow:
        1. Start QR detection
        2. Send waypoints to Nav2
        3. Monitor progress
        4. Complete
        """
        
        if self.state == MissionState.IDLE:
            if self.waypoints:
                self.get_logger().info('')
                self.get_logger().info('🚀 Starting inventory mission...')
                self.state = MissionState.STARTING_QR
        
        elif self.state == MissionState.STARTING_QR:
            if not self.qr_detection_started:
                self.get_logger().info('')
                self.get_logger().info('=' * 70)
                self.get_logger().info('  MISSION INITIALIZATION')
                self.get_logger().info('=' * 70)
                self.get_logger().info('[1/2] Starting QR detection system...')
                self.enable_qr_detection(True)
                self.qr_detection_started = True
                time.sleep(2)  # Let QR system initialize
                
                self.get_logger().info('✓ QR detection running continuously')
                self.get_logger().info('[2/2] Waiting for Nav2 waypoint_follower server...')
                
                # Wait for Nav2 server
                if self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().info('✓ Nav2 waypoint_follower ready')
                    self.get_logger().info('=' * 70)
                    self.get_logger().info('')
                    self.state = MissionState.SENDING_WAYPOINTS
                else:
                    self.get_logger().error('❌ Nav2 waypoint_follower server not available!')
                    self.get_logger().error('   Is Nav2 running?')
                    self.state = MissionState.ERROR
        
        elif self.state == MissionState.SENDING_WAYPOINTS:
            self.send_follow_waypoints_goal()
            self.state = MissionState.NAVIGATING
        
        elif self.state == MissionState.NAVIGATING:
            # Nav2 is handling navigation, we just wait
            pass
        
        elif self.state == MissionState.MISSION_COMPLETE:
            # Mission done, just idle
            pass
        
        elif self.state == MissionState.ERROR:
            # Error state, stop mission
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = Nav2WaypointFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('')
            node.get_logger().info('🛑 Mission stopped by user')
    except Exception as e:
        if node:
            node.get_logger().error(f'❌ Error: {e}')
        else:
            print(f'❌ Failed to initialize node: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
