#!/usr/bin/env python3
"""
Waypoint Path Visualizer Node
Reads waypoints from JSON and publishes RViz markers for visualization

Features:
- Publishes waypoints as sphere markers
- Publishes path connecting waypoints
- Publishes orientation arrows
- Displays waypoint IDs as text labels
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import os
from enum import Enum


class Colors(Enum):
    RED = (1.0, 0.0, 0.0)
    GREEN = (0.0, 1.0, 0.0)
    BLUE = (0.0, 0.0, 1.0)
    YELLOW = (1.0, 1.0, 0.0)
    CYAN = (0.0, 1.0, 1.0)
    MAGENTA = (1.0, 0.0, 1.0)
    WHITE = (1.0, 1.0, 1.0)
    ORANGE = (1.0, 0.5, 0.0)


class WaypointPathVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_path_visualizer')
        
        # Declare parameters
        self.declare_parameter('waypoints_file', '')
        
        # Get parameters
        waypoints_file = self.get_parameter('waypoints_file').value
        waypoints_file = os.path.expanduser(waypoints_file)
        
        # Load waypoints
        self.waypoints = []
        self.load_waypoints(waypoints_file)
        
        # Publishers
        self.markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Publish markers once at startup
        self.publish_markers()
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('WAYPOINT PATH VISUALIZER INITIALIZED')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Published {len(self.waypoints)} waypoint markers to RViz')
        self.get_logger().info('View in RViz: Add MarkerArray display at /waypoint_markers')
        self.get_logger().info('=' * 80)
    
    def load_waypoints(self, waypoints_file):
        """Load waypoints from JSON file"""
        try:
            if not os.path.exists(waypoints_file):
                self.get_logger().error(f'Waypoints file not found: {waypoints_file}')
                return
            
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
            
            self.waypoints = data.get('waypoints', [])
            self.get_logger().info(f'✅ Loaded {len(self.waypoints)} waypoints')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
    
    def publish_markers(self):
        """Publish all waypoint markers"""
        if len(self.waypoints) == 0:
            return
        
        marker_array = MarkerArray()
        
        # Create path line (connecting waypoints)
        path_marker = self.create_path_marker()
        marker_array.markers.append(path_marker)
        
        # Create waypoint spheres and labels
        for i, wp in enumerate(self.waypoints):
            # Sphere marker for waypoint position
            sphere_marker = self.create_waypoint_marker(wp, i)
            marker_array.markers.append(sphere_marker)
            
            # Text label
            text_marker = self.create_text_marker(wp, i)
            marker_array.markers.append(text_marker)
            
            # Orientation arrow
            arrow_marker = self.create_arrow_marker(wp, i)
            marker_array.markers.append(arrow_marker)
        
        # Publish all markers
        self.markers_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} markers')
    
    def create_path_marker(self):
        """Create a line connecting all waypoints"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoint_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Line style
        marker.scale.x = 0.05  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # Semi-transparent green
        
        # Add all waypoints to the line
        for wp in self.waypoints:
            point = Point()
            point.x = wp['position']['x']
            point.y = wp['position']['y']
            point.z = wp['position']['z']
            marker.points.append(point)
        
        return marker
    
    def create_waypoint_marker(self, waypoint, index):
        """Create a sphere marker for a waypoint"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = waypoint['id']
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = waypoint['position']['x']
        marker.pose.position.y = waypoint['position']['y']
        marker.pose.position.z = waypoint['position']['z']
        
        # Size
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Color: first waypoint is green, last is red, middle are blue
        if index == 0:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif index == len(self.waypoints) - 1:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        
        marker.color.a = 0.9
        
        return marker
    
    def create_text_marker(self, waypoint, index):
        """Create a text marker with waypoint ID"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoint_labels'
        marker.id = waypoint['id']
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position (slightly above the waypoint)
        marker.pose.position.x = waypoint['position']['x']
        marker.pose.position.y = waypoint['position']['y']
        marker.pose.position.z = waypoint['position']['z'] + 0.3
        
        # Text
        marker.text = f"WP{waypoint['id']}"
        
        # Style
        marker.scale.z = 0.2  # Text height
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker
    
    def create_arrow_marker(self, waypoint, index):
        """Create an arrow showing waypoint orientation"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoint_arrows'
        marker.id = waypoint['id']
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = waypoint['position']['x']
        marker.pose.position.y = waypoint['position']['y']
        marker.pose.position.z = waypoint['position']['z']
        
        # Orientation (from quaternion)
        marker.pose.orientation.x = waypoint['orientation']['x']
        marker.pose.orientation.y = waypoint['orientation']['y']
        marker.pose.orientation.z = waypoint['orientation']['z']
        marker.pose.orientation.w = waypoint['orientation']['w']
        
        # Size
        marker.scale.x = 0.3  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1  # Arrow height
        
        # Color: orange
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = WaypointPathVisualizer()
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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
