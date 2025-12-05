#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from enum import Enum
import math
import time

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


class MissionState(Enum):
    IDLE = 0
    NAVIGATING_TO_RACK = 1
    ADJUSTING_LIFT = 2
    SCANNING_QR = 3
    MOVING_TO_NEXT_SHELF = 4
    RACK_COMPLETE = 5
    MISSION_COMPLETE = 6
    FAILED = 7


class RackWaypoint:
    def __init__(self, name, x, y, theta, shelf_heights):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.shelf_heights = shelf_heights


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # State variables
        self.state = MissionState.IDLE
        self.current_rack_idx = 0
        self.current_shelf_idx = 0
        self.qr_detected_flag = False
        self.waypoints = []
        
        # Parameters
        self.declare_parameter('scan_dwell_time', 3.0)
        self.declare_parameter('lift_move_time', 2.0)
        
        self.scan_dwell_time = self.get_parameter('scan_dwell_time').value
        self.lift_move_time = self.get_parameter('lift_move_time').value
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        # self.lift_pub = self.create_publisher(
        #     JointTrajectory,
        #     '/lift_position_controller/joint_trajectory',
        #     10
        # )
        
        self.lift_pub = self.create_publisher(
             Float64MultiArray,
            '/qr_camera_controller/commands',
            10
        )
        
        
        self.get_logger().info('lift_pub created')

        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/mission_waypoints_markers',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Subscribers
        self.qr_sub = self.create_subscription(
            String,
            '/qr_detections/status',
            self.qr_detection_callback,
            10
        )
        
        # Timers
        self.marker_timer = self.create_timer(1.0, self.publish_waypoint_markers)
        self.mission_timer = None
        
        # Load waypoints
        self.load_waypoints()
        self.publish_waypoint_markers()
        
        self.get_logger().info('════════════════════════════════════════')
        self.get_logger().info('  Mission Controller Started')
        self.get_logger().info('════════════════════════════════════════')
        self.get_logger().info(f'Loaded {len(self.waypoints)} rack waypoints')
        self.get_logger().info(f'Scan dwell time: {self.scan_dwell_time:.1f}s')
        self.get_logger().info('Waiting for Nav2 action server...')
        
        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            self.get_logger().warn('Make sure Nav2 is running!')
        else:
            self.get_logger().info('Nav2 connected! Ready to start mission.')
    
    def start_oscillation(self):
        self.get_logger().info('Starting oscillation...')
        
        back_distance = 0.62
        front_distance = 0.69
        speed = 0.1
        front_duration = front_distance / speed
        back_duration = back_distance / speed
        
        cmd = Twist()
        
        # Move backward
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < back_duration:
            cmd.linear.x = -speed
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        
        # Move forward
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < front_duration:
            cmd.linear.x = speed
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        
        # Stop
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.2)
        
        # Final stop
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('Oscillation complete. Starting scan dwell...')
        
        self.state = MissionState.SCANNING_QR
        self.qr_detected_flag = False
    
    def load_waypoints(self):
        """Load measured rack positions"""
        # RACK 1
        self.get_logger().info('Loading rack waypoints from map measurements...')
        r1 = RackWaypoint(
            name='RACK_1',
            x=-1.23,
            y=3.8,
            theta=0.0 * math.pi / 180.0,
            shelf_heights=[0.0, 0.50, 0.75, 1.15]
        )
        self.waypoints.append(r1)
        
        # RACK 2
        r2 = RackWaypoint(
            name='RACK_2',
            x=-0.18,
            y=3.8,
            theta=0.0 * math.pi / 180.0,
            shelf_heights=[0.0, 0.50, 0.75, 1.15]
        )
        self.waypoints.append(r2)
        
        # RACK 3
        r3 = RackWaypoint(
            name='RACK_3',
            x=-0.322,
            y=3.554,
            theta=270.0 * math.pi / 180.0,
            shelf_heights=[0.0, 0.50, 0.75, 1.15]
        )
        self.waypoints.append(r3)
        
        # RACK 4
        r4 = RackWaypoint(
            name='RACK_4',
            x=-0.348,
            y=2.6,
            theta=270.0 * math.pi / 180.0,
            shelf_heights=[0.0, 0.50, 0.75, 1.15]
        )
        self.waypoints.append(r4)
        
        # RACK 5
        r5 = RackWaypoint(
            name='RACK_5',
            x=-0.25,
            y=1.463,
            theta=270.0 * math.pi / 180.0,
            shelf_heights=[0.0, 0.50, 0.75, 1.15]
        )
        self.waypoints.append(r5)
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} racks from map measurements')
    
    def start_mission(self):
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded!')
            return
        
        self.get_logger().info('════════════════════════════════════════')
        self.get_logger().info('  STARTING WAREHOUSE SCAN MISSION')
        self.get_logger().info('════════════════════════════════════════')
        self.get_logger().info(f'Total racks: {len(self.waypoints)}')
        self.get_logger().info(f'Total shelves: {len(self.waypoints[0].shelf_heights)} per rack')
        
        self.current_rack_idx = 0
        self.current_shelf_idx = 0
        self.state = MissionState.IDLE
        
        self.execute_next_rack()
    
    def execute_next_rack(self):
        if self.current_rack_idx >= len(self.waypoints):
            self.state = MissionState.MISSION_COMPLETE
            self.get_logger().info('════════════════════════════════════════')
            self.get_logger().info('  MISSION COMPLETE!')
            self.get_logger().info('════════════════════════════════════════')
            self.get_logger().info(f'Scanned {len(self.waypoints)} racks successfully')
            return
        
        rack = self.waypoints[self.current_rack_idx]
        
        self.get_logger().info('════════════════════════════════════════')
        self.get_logger().info(f'  RACK {self.current_rack_idx + 1}/{len(self.waypoints)}: {rack.name}')
        self.get_logger().info('════════════════════════════════════════')
        
        self.current_shelf_idx = 0
        self.navigate_to_rack(rack)
    
    def navigate_to_rack(self, rack):
        self.state = MissionState.NAVIGATING_TO_RACK
        
        self.get_logger().info(f'Navigating to: ({rack.x:.2f}, {rack.y:.2f}) theta={rack.theta * 180.0 / math.pi:.0f} deg')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = rack.x
        goal_msg.pose.pose.position.y = rack.y
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.z = math.sin(rack.theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(rack.theta / 2.0)
        
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def publish_waypoint_markers(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        for rack in self.waypoints:
            # Sphere for waypoint position
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoints'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            
            sphere.pose.position.x = rack.x
            sphere.pose.position.y = rack.y
            sphere.pose.position.z = 0.1
            
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            
            sphere.color.r = 1.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            
            sphere.lifetime = Duration(seconds=0).to_msg()
            marker_array.markers.append(sphere)
            
            # Arrow for orientation
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'waypoints'
            arrow.id = marker_id
            marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            arrow.pose.position.x = rack.x
            arrow.pose.position.y = rack.y
            arrow.pose.position.z = 0.1
            
            half = rack.theta / 2.0
            arrow.pose.orientation.x = 0.0
            arrow.pose.orientation.y = 0.0
            arrow.pose.orientation.z = math.sin(half)
            arrow.pose.orientation.w = math.cos(half)
            
            arrow.scale.x = 0.5  # length
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 1.0
            arrow.color.a = 1.0
            
            arrow.lifetime = Duration(seconds=0).to_msg()
            marker_array.markers.append(arrow)
            
            # Text label
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'waypoints_text'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            
            text.pose.position.x = rack.x
            text.pose.position.y = rack.y
            text.pose.position.z = 0.5
            
            text.scale.z = 0.25
            
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            
            text.text = rack.name
            
            text.lifetime = Duration(seconds=0).to_msg()
            marker_array.markers.append(text)
        
        self.marker_pub.publish(marker_array)
    
    def move_lift_to_height(self, height):
        self.get_logger().info(f'Moving lift to {height:.2f} m')
        
        self.state = MissionState.ADJUSTING_LIFT
        
        traj = JointTrajectory()
        traj.joint_names = ['lift_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = Duration(seconds=self.lift_move_time).to_msg()
        traj.points.append(point)
        
        self.lift_pub.publish(traj)
        
        wait_time = 3.0  # allow lift to settle
        
        # Cancel previous timer if exists
        if self.mission_timer is not None:
            self.mission_timer.cancel()
        
        # Create timer for oscillation and next shelf processing
        self.mission_timer = self.create_timer(
            wait_time,
            self.on_lift_complete
        )
    
    def on_lift_complete(self):
        if self.mission_timer is not None:
            self.mission_timer.cancel()
            self.mission_timer = None
        self.start_oscillation()
        self.process_next_shelf()
    
    def wait_for_qr_detection(self):
        # This function is no longer used - scanning happens via timer
        pass
    
    def process_next_shelf(self):
        rack = self.waypoints[self.current_rack_idx]
        
        if self.qr_detected_flag:
            self.get_logger().info('  ✓ QR code detected!')
        else:
            self.get_logger().warn('  ⚠ No QR code detected')
        
        self.current_shelf_idx += 1
        
        if self.current_shelf_idx >= len(rack.shelf_heights):
            self.state = MissionState.RACK_COMPLETE
            self.get_logger().info(f'✓ Rack {rack.name} complete!')
            
            self.current_rack_idx += 1
            self.execute_next_rack()
        else:
            next_height = rack.shelf_heights[self.current_shelf_idx]
            self.move_lift_to_height(next_height)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.state = MissionState.FAILED
            return
        
        self.get_logger().info('  Navigation goal accepted')
        goal_handle.get_result_async().add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        self.get_logger().debug(
            f'  Position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})'
        )
    
    def result_callback(self, future):
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Arrived at rack!')
            
            if self.waypoints:
                first_height = self.waypoints[self.current_rack_idx].shelf_heights[0]
                self.move_lift_to_height(first_height)
        elif result.status == 5:  # ABORTED
            self.get_logger().error('Navigation aborted!')
            self.state = MissionState.FAILED
        elif result.status == 6:  # CANCELED
            self.get_logger().warn('Navigation canceled')
            self.state = MissionState.FAILED
        else:
            self.get_logger().error('Unknown navigation result')
            self.state = MissionState.FAILED
    
    def qr_detection_callback(self, msg):
        if self.state == MissionState.SCANNING_QR:
            if msg.data == 'detected':
                self.qr_detected_flag = True


def main(args=None):
    rclpy.init(args=args)
    
    node = MissionController()
    node.start_mission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
