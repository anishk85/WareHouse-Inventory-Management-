#!/usr/bin/env python3
"""
Waypoint Manager Node
Manages waypoints for warehouse navigation - save, load, and provide waypoints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from warehouse_rover_msgs.srv import SaveWaypoint, GetWaypoints
import json
import os
import math
from tf_transformations import euler_from_quaternion


class WaypointManager(Node):
    """
    Manages waypoints for autonomous navigation
    
    Services:
        ~/save_waypoint: Save current robot pose as waypoint
        ~/get_waypoints: Get list of saved waypoints
    
    Subscriptions:
        /amcl_pose: Current robot pose from AMCL
    """
    
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # Declare parameters
        self.declare_parameter('waypoints_file', '/tmp/warehouse_waypoints.json')
        self.declare_parameter('auto_save', True)
        
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.auto_save = self.get_parameter('auto_save').value
        
        # Waypoints storage
        self.waypoints = {}  # {name: {x, y, yaw, type}}
        self.current_pose = None
        
        # Load existing waypoints
        self.load_waypoints()
        
        # Subscribe to robot pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Create services
        self.save_srv = self.create_service(
            SaveWaypoint,
            '~/save_waypoint',
            self.save_waypoint_callback
        )
        
        self.get_srv = self.create_service(
            GetWaypoints,
            '~/get_waypoints',
            self.get_waypoints_callback
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Waypoint Manager Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Waypoints file: {self.waypoints_file}')
        self.get_logger().info(f'Loaded waypoints: {len(self.waypoints)}')
        self.get_logger().info('Services:')
        self.get_logger().info('  ~/save_waypoint - Save current pose')
        self.get_logger().info('  ~/get_waypoints - Get saved waypoints')
        self.get_logger().info('=' * 60)
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store current robot pose"""
        self.current_pose = msg.pose.pose
    
    def save_waypoint_callback(self, request, response):
        """Save current pose as a waypoint"""
        if self.current_pose is None:
            response.success = False
            response.message = "No pose data available yet. Wait for AMCL localization."
            return response
        
        # Extract position
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Extract yaw from quaternion
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        
        # Save waypoint
        self.waypoints[request.waypoint_name] = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'type': request.waypoint_type or 'checkpoint'
        }
        
        # Auto-save to file
        if self.auto_save:
            self.save_waypoints()
        
        response.success = True
        response.message = f"Waypoint '{request.waypoint_name}' saved"
        response.x = x
        response.y = y
        response.yaw = yaw
        
        self.get_logger().info(
            f"✓ Saved waypoint '{request.waypoint_name}' "
            f"at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)"
        )
        
        return response
    
    def get_waypoints_callback(self, request, response):
        """Get waypoints"""
        if request.waypoint_name:
            # Get specific waypoint
            if request.waypoint_name in self.waypoints:
                wp = self.waypoints[request.waypoint_name]
                response.waypoint_names = [request.waypoint_name]
                response.x_positions = [wp['x']]
                response.y_positions = [wp['y']]
                response.yaw_orientations = [wp['yaw']]
                response.waypoint_types = [wp['type']]
                response.success = True
                response.message = f"Found waypoint '{request.waypoint_name}'"
            else:
                response.success = False
                response.message = f"Waypoint '{request.waypoint_name}' not found"
        else:
            # Get all waypoints
            response.waypoint_names = list(self.waypoints.keys())
            response.x_positions = [wp['x'] for wp in self.waypoints.values()]
            response.y_positions = [wp['y'] for wp in self.waypoints.values()]
            response.yaw_orientations = [wp['yaw'] for wp in self.waypoints.values()]
            response.waypoint_types = [wp['type'] for wp in self.waypoints.values()]
            response.success = True
            response.message = f"Found {len(self.waypoints)} waypoints"
        
        return response
    
    def load_waypoints(self):
        """Load waypoints from file"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    self.waypoints = json.load(f)
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from file")
            except Exception as e:
                self.get_logger().error(f"Failed to load waypoints: {e}")
        else:
            self.get_logger().info("No existing waypoints file found")
    
    def save_waypoints(self):
        """Save waypoints to file"""
        try:
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            with open(self.waypoints_file, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to file")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e}")
    
    def shutdown(self):
        """Save waypoints on shutdown"""
        if self.auto_save:
            self.save_waypoints()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = WaypointManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt')
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
