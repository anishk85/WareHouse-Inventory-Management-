#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.waypoints = []
        
        # --- CRITICAL CHANGE ---
        # We listen to a CUSTOM topic, not the default Nav2 topic ('/goal_pose').
        # This ensures the robot's navigation stack (Nav2) ignores these clicks.
        self.custom_topic = '/recorded_pose'
        
        # Create subscription
        self.subscription = self.create_subscription(
            PoseStamped,
            self.custom_topic,
            self.listener_callback,
            10  # QoS depth
        )
        
        print(f"Waypoint Logger Node Started: {self.get_name()}")
        print("---------------------------------------------------")
        print("CRITICAL: You must change RViz 2 settings for this to work!")
        print("---------------------------------------------------")
        print("INSTRUCTIONS:")
        print("1. Open RViz 2.")
        print("2. Look at the 'Tools' panel (usually on the left or top).")
        print("   If you don't see the tool properties, go to Panels -> Tool Properties.")
        print("3. Find the '2D Goal Pose' tool (or the arrow tool you use for navigation).")
        print(f"4. Change the 'Topic' field from '/goal_pose' (or similar) to '{self.custom_topic}'.")
        print("5. Now, use the '2D Goal Pose' arrow to click points.")
        print("   The robot will NOT move, but the point will be captured here.")
        print("6. Press Ctrl+C in this terminal when done to get your array.")
        print("---------------------------------------------------")

    def listener_callback(self, msg):
        # Extract position
        p = msg.pose.position
        # Extract orientation
        o = msg.pose.orientation
        
        # Store simplified data
        waypoint = {
            'x': p.x, 'y': p.y, 'z': p.z,
            'qx': o.x, 'qy': o.y, 'qz': o.z, 'qw': o.w
        }
        
        self.waypoints.append(waypoint)
        
        print(f"Captured Point {len(self.waypoints)}: x={p.x:.2f}, y={p.y:.2f} Orientation (quat)=({o.x:.2f}, {o.y:.2f}, {o.z:.2f}, {o.w:.2f})")

    def print_waypoints(self):
        print("\n\n" + "="*50)
        print(" FINAL WAYPOINTS ARRAY ")
        print(" Copy this list into your navigation script:")
        print("="*50)
        print("waypoints = [")
        for i, wp in enumerate(self.waypoints):
            comma = "," if i < len(self.waypoints) - 1 else ""
            print(f"    {{'x': {wp['x']:.4f}, 'y': {wp['y']:.4f}, 'yaw_quat': [{wp['qx']:.4f}, {wp['qy']:.4f}, {wp['qz']:.4f}, {wp['qw']:.4f}]}}{comma}")
        print("]")
        print("="*50)
        print("NOTE: Don't forget to change the RViz 2 topic back to '/goal_pose'")
        print("      (or whatever your Nav2 stack uses) to command the robot normally again!")

def main(args=None):
    rclpy.init(args=args)
    logger = WaypointLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.print_waypoints()
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()