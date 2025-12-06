#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml # Import the YAML library
import os   # Import os for file path manipulation

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.waypoints = []
        
        self.output_filename = 'recorded_waypoints.yaml' 
        
        self.custom_topic = '/recorded_pose'
        
        self.subscription = self.create_subscription(
            PoseStamped,
            self.custom_topic,
            self.listener_callback,
            10  # QoS depth
        )
        
        print(f"Waypoint Logger Node Started: {self.get_name()}")
        print("---------------------------------------------------")
        print("INSTRUCTIONS (RViz 2):")
        print("1. Find the '2D Goal Pose' tool.")
        print(f"2. Change its 'Topic' field to '{self.custom_topic}'.")
        print("3. Use the arrow tool to click points.")
        print("4. Press Ctrl+C to stop and save the file.")
        print("---------------------------------------------------")

    def listener_callback(self, msg):
        # Extract position and orientation
        p = msg.pose.position
        o = msg.pose.orientation
        
        # Store simplified data in a dictionary that mirrors the YAML structure
        waypoint = {
            'x': p.x, 
            'y': p.y, 
            'z': p.z,
            'yaw_quat': [o.x, o.y, o.z, o.w]
        }
        
        self.waypoints.append(waypoint)
        
        print(f"Captured Point {len(self.waypoints)}: x={p.x:.2f}, y={p.y:.2f}")

    def save_waypoints_to_file(self):
        """Writes the captured waypoints to a YAML file."""
        
        # Prepare the final data structure for the YAML file
        data = {
            'waypoints': self.waypoints
        }
        
        try:
            with open(self.output_filename, 'w') as f:
                # Use safe_dump for security and standard YAML formatting
                yaml.safe_dump(data, f, indent=4) 
            
            # Use os.path.abspath to print the full file path for easy access
            full_path = os.path.abspath(self.output_filename)
            
            print("\n\n" + "="*70)
            print(" ✅ WAYPOINTS SUCCESSFULLY SAVED TO FILE! ")
            print("="*70)
            print(f"File: **{self.output_filename}**")
            print(f"Path: {full_path}")
            print("---------------------------------------------------")
            print("NOTE: Other nodes can load this file using the 'yaml' library.")

        except Exception as e:
            self.get_logger().error(f"Failed to write waypoints to file: {e}")

def main(args=None):
    rclpy.init(args=args)
    logger = WaypointLogger()
    
    try:
        rclpy.spin(logger)
        
    except KeyboardInterrupt:
        pass
        
    finally:
        logger.save_waypoints_to_file()
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()