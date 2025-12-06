#!/usr/bin/env python3
"""
Interactive tool to measure rack positions from RViz
Click "Publish Point" on each rack center in RViz
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class RackPositionMeasurer(Node):
    def __init__(self):
        super().__init__('rack_position_measurer')
        
        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )
        
        self.rack_count = 0
        
        print("\n" + "="*60)
        print("  RACK POSITION MEASUREMENT TOOL")
        print("="*60)
        print("\nInstructions:")
        print("1. Open RViz")
        print("2. Click 'Publish Point' button at top")
        print("3. Click on CENTER of each rack")
        print("4. Note the orientation (which way rack faces)")
        print("5. Copy YAML output below")
        print("\nWaiting for clicks...\n")
    
    def point_callback(self, msg):
        self.rack_count += 1
        
        x = msg.point.x
        y = msg.point.y
        
        print("="*60)
        print(f"  RACK {self.rack_count} DETECTED")
        print("="*60)
        print(f"Position: ({x:.3f}, {y:.3f})")
        print("\nYAML format (update theta based on rack orientation):")
        print(f"""
RackWaypoint r{self.rack_count};
r{self.rack_count}.name = "RACK_{self.rack_count}";
r{self.rack_count}.x = {x:.3f};
r{self.rack_count}.y = {y:.3f};
r{self.rack_count}.theta = 0.0;  // UPDATE: 0, 90, 180, or 270 degrees
r{self.rack_count}.shelf_heights = {{0.0, 0.25, 0.50, 0.75}};
waypoints_.push_back(r{self.rack_count});
""")
        print("="*60 + "\n")


def main():
    rclpy.init()
    node = RackPositionMeasurer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
