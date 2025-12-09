#!/usr/bin/env python3
"""
Waypoint Recorder Node
Records robot poses (position + orientation) during mapping/teleoperation
Saves waypoints to a local JSON file with timestamps and metadata

Usage:
- Press 'SPACE' key or publish to /waypoint/record topic to record current pose
- Waypoints are automatically saved to ~/.ros/waypoints/waypoints_YYYYMMDD_HHMMSS.json
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
import json
import os
from datetime import datetime
import math


class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')
        
        # Declare parameters
        self.declare_parameter('waypoint_file', '')  # Auto-generate if empty
        self.declare_parameter('waypoint_dir', os.path.expanduser('~/.ros/waypoints'))
        self.declare_parameter('use_odom', True)  # Use odometry instead of TF
        self.declare_parameter('odom_topic', '/mecanum_drive_controller/odometry')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        # Get parameters
        self.waypoint_dir = self.get_parameter('waypoint_dir').value
        self.use_odom = self.get_parameter('use_odom').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Create waypoint directory if it doesn't exist
        os.makedirs(self.waypoint_dir, exist_ok=True)
        
        # Generate waypoint filename
        waypoint_file = self.get_parameter('waypoint_file').value
        if not waypoint_file:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            waypoint_file = f'waypoints_{timestamp}.json'
        
        self.waypoint_filepath = os.path.join(self.waypoint_dir, waypoint_file)
        
        # Initialize waypoint storage
        self.waypoints = []
        self.waypoint_count = 0
        
        # TF Buffer and Listener (for getting map->base_link transform)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Current pose storage
        self.current_pose = None
        self.current_odom = None
        
        # Subscribe to odometry for pose
        if self.use_odom:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                10
            )
        
        # Subscribe to record trigger
        self.record_sub = self.create_subscription(
            Bool,
            'waypoint/record',
            self.record_callback,
            10
        )
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, 'waypoint/status', 10)
        
        # Publisher for waypoint markers (for RViz visualization)
        self.marker_pub = self.create_publisher(PoseStamped, 'waypoint/current', 10)
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('WAYPOINT RECORDER NODE INITIALIZED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Waypoint file: {self.waypoint_filepath}')
        self.get_logger().info(f'Using {"odometry" if self.use_odom else "TF"} for pose')
        if self.use_odom:
            self.get_logger().info(f'Odometry topic: {self.odom_topic}')
        else:
            self.get_logger().info(f'TF frames: {self.map_frame} -> {self.base_frame}')
        self.get_logger().info('')
        self.get_logger().info('RECORDING METHODS:')
        self.get_logger().info('  1. Publish Bool(True) to /waypoint/record')
        self.get_logger().info('  2. Use teleop key mapping (if configured)')
        self.get_logger().info('')
        self.get_logger().info('Press trigger to record waypoints!')
        self.get_logger().info('=' * 70)
    
    def odom_callback(self, msg):
        """Store current odometry data"""
        self.current_odom = msg
        self.current_pose = msg.pose.pose
    
    def get_current_pose(self):
        """Get current robot pose (from odom or TF)"""
        if self.use_odom:
            if self.current_pose is None:
                self.get_logger().warn('No odometry data received yet')
                return None
            return self.current_pose
        else:
            # Get transform from map to base_link
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
                
                # Convert transform to pose
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                
                return pose
            except TransformException as ex:
                self.get_logger().warn(f'Could not get transform: {ex}')
                return None
    
    def record_callback(self, msg):
        """Handle waypoint recording trigger"""
        if msg.data:
            self.record_waypoint()
    
    def record_waypoint(self):
        """Record current pose as a waypoint"""
        pose = self.get_current_pose()
        
        if pose is None:
            self.get_logger().error('❌ Cannot record waypoint - no pose available')
            return
        
        self.waypoint_count += 1
        
        # Extract position and orientation
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        # Convert quaternion to yaw angle (for easier reading)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                        1.0 - 2.0 * (qy * qy + qz * qz))
        yaw_deg = math.degrees(yaw)
        
        # Create waypoint entry
        waypoint = {
            'id': self.waypoint_count,
            'name': f'waypoint_{self.waypoint_count}',
            'timestamp': datetime.now().isoformat(),
            'position': {
                'x': round(x, 3),
                'y': round(y, 3),
                'z': round(z, 3)
            },
            'orientation': {
                'x': round(qx, 4),
                'y': round(qy, 4),
                'z': round(qz, 4),
                'w': round(qw, 4)
            },
            'yaw_degrees': round(yaw_deg, 2),
            'metadata': {
                'frame_id': self.map_frame if not self.use_odom else 'odom',
                'recording_method': 'odometry' if self.use_odom else 'tf'
            }
        }
        
        # Add to waypoint list
        self.waypoints.append(waypoint)
        
        # Save to file
        self.save_waypoints()
        
        # Log success
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'✅ WAYPOINT {self.waypoint_count} RECORDED')
        self.get_logger().info('─' * 70)
        self.get_logger().info(f'Position:    x={x:7.3f}m  y={y:7.3f}m  z={z:7.3f}m')
        self.get_logger().info(f'Orientation: yaw={yaw_deg:6.2f}° (qx={qx:.3f} qy={qy:.3f} qz={qz:.3f} qw={qw:.3f})')
        self.get_logger().info(f'Saved to:    {self.waypoint_filepath}')
        self.get_logger().info('=' * 70)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'recorded_waypoint_{self.waypoint_count}'
        self.status_pub.publish(status_msg)
        
        # Publish marker for visualization
        self.publish_marker(pose)
    
    def save_waypoints(self):
        """Save all waypoints to JSON file"""
        data = {
            'metadata': {
                'created': datetime.now().isoformat(),
                'total_waypoints': len(self.waypoints),
                'robot_frame': self.base_frame,
                'map_frame': self.map_frame if not self.use_odom else 'odom',
                'recording_method': 'odometry' if self.use_odom else 'tf'
            },
            'waypoints': self.waypoints
        }
        
        try:
            with open(self.waypoint_filepath, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().debug(f'Saved {len(self.waypoints)} waypoints to {self.waypoint_filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
    
    def publish_marker(self, pose):
        """Publish pose marker for RViz visualization"""
        marker = PoseStamped()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame if not self.use_odom else 'odom'
        marker.pose = pose
        self.marker_pub.publish(marker)
    
    def publish_status(self):
        """Publish periodic status update"""
        if self.waypoint_count > 0:
            status_msg = String()
            status_msg.data = f'recording_active_{self.waypoint_count}_waypoints'
            self.status_pub.publish(status_msg)
    
    def on_shutdown(self):
        """Called when node is shutting down"""
        self.get_logger().info('=' * 70)
        self.get_logger().info('WAYPOINT RECORDER SHUTTING DOWN')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Total waypoints recorded: {self.waypoint_count}')
        self.get_logger().info(f'Waypoints saved to: {self.waypoint_filepath}')
        self.get_logger().info('=' * 70)


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = WaypointRecorderNode()
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
            node.on_shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()