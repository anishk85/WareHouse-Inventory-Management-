#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import os
import sys
import yaml

# --- CONFIGURATION ---
YAML_FILENAME = 'maps.yaml' 
OFFSET_METERS = 1.0

# ==========================================
# PART 1: CALCULATE GOAL POINTS
# ==========================================

def get_map_metadata(yaml_path):
    if not os.path.exists(yaml_path):
        print(f"ERROR: Map YAML not found at {yaml_path}")
        sys.exit(1)
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
    return map_data['resolution'], map_data['origin'], os.path.join(os.path.dirname(os.path.abspath(yaml_path)), map_data['image'])

def get_offset_segment(p1, p2, center_point, offset_dist):
    p1, p2, center = np.array(p1, float), np.array(p2, float), np.array(center_point, float)
    wall_vec = p2 - p1
    ortho_vec = np.array([-wall_vec[1], wall_vec[0]]) 
    if np.linalg.norm(ortho_vec) == 0: return tuple(p1.astype(int)), tuple(p2.astype(int))
    ortho_vec = ortho_vec / np.linalg.norm(ortho_vec)
    if np.dot(ortho_vec, center - (p1+p2)/2.0) < 0: ortho_vec = -ortho_vec
    shift = ortho_vec * offset_dist
    return (p1 + shift), (p2 + shift)

def get_line_intersection(line1_p1, line1_p2, line2_p1, line2_p2):
    x1, y1, x2, y2 = *line1_p1, *line1_p2
    x3, y3, x4, y4 = *line2_p1, *line2_p2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0: return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
    return (int(px), int(py))

def pixel_to_world(px, py, img_height, res, origin):
    return (px * res) + origin[0], ((img_height - py) * res) + origin[1]

def get_target_goals():
    print("--- Analyzing Map ---")
    res, origin, img_path = get_map_metadata(YAML_FILENAME)
    img = cv2.imread(img_path)
    if img is None: sys.exit(1)
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: sys.exit(1)
    
    main_wall = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(main_wall)
    corners = np.intp(cv2.boxPoints(rect))
    room_center = np.mean(corners, axis=0)
    
    # Find Entrance
    entrance_center = None
    if len(main_wall) > 3:
        hull = cv2.convexHull(main_wall, returnPoints=False)
        defects = cv2.convexityDefects(main_wall, hull)
        if defects is not None:
            max_depth = 0
            best = None
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                if (d/256.0) > max_depth:
                    max_depth = d/256.0
                    best = (main_wall[s][0], main_wall[e][0])
            if best: entrance_center = np.mean(best, axis=0)
            
    if entrance_center is None: 
        print("Entrance not found."); sys.exit(1)

    # Identify Walls
    V_in = room_center - entrance_center
    walls = []
    for i in range(4):
        p1, p2 = corners[i], corners[(i+1)%4]
        mid = (p1+p2)/2.0
        walls.append({'seg': (p1, p2), 'dist': np.linalg.norm(mid - entrance_center), 'cp': np.cross(V_in, mid - room_center)})
    walls.sort(key=lambda x: x['dist'])
    
    # Rack 2 (Front) & Rack 1 (Right)
    rack2 = walls[3]['seg']
    rack1 = walls[1]['seg'] if walls[1]['cp'] > walls[2]['cp'] else walls[2]['seg']
    
    # Calculate Intersections and Offsets
    off_px = int(OFFSET_METERS / res)
    r1_p1, r1_p2 = get_offset_segment(rack1[0], rack1[1], room_center, off_px)
    r2_p1, r2_p2 = get_offset_segment(rack2[0], rack2[1], room_center, off_px)
    
    intersection_px = get_line_intersection(r1_p1, r1_p2, r2_p1, r2_p2)
    
    goals_world = []

    if intersection_px:
        # 1. Determine which end of Rack 1 offset is the "Far End"
        # We compare distances from the intersection point
        dist1 = np.linalg.norm(np.array(r1_p1) - np.array(intersection_px))
        dist2 = np.linalg.norm(np.array(r1_p2) - np.array(intersection_px))
        
        # The point with the larger distance is the "Far End"
        if dist1 > dist2:
            far_end_px = tuple(r1_p1.astype(int))
        else:
            far_end_px = tuple(r1_p2.astype(int))
            
        # Debug Visualization
        cv2.line(img, far_end_px, intersection_px, (255, 0, 255), 2) # Draw path
        cv2.circle(img, far_end_px, 8, (0, 165, 255), -1)   # Orange = Start (Far End)
        cv2.circle(img, intersection_px, 8, (0, 255, 255), -1) # Yellow = End (Intersection)
        cv2.putText(img, "1. Start", far_end_px, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        cv2.putText(img, "2. End", intersection_px, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        
        cv2.imwrite('debug_goals.png', img)
        print("Debug image saved: 'debug_goals.png'")
        
        # Convert to World Coordinates
        # Goal 1: Far End
        g1_x, g1_y = pixel_to_world(far_end_px[0], far_end_px[1], img.shape[0], res, origin)
        goals_world.append((g1_x, g1_y))
        
        # Goal 2: Intersection
        g2_x, g2_y = pixel_to_world(intersection_px[0], intersection_px[1], img.shape[0], res, origin)
        goals_world.append((g2_x, g2_y))
        
        return goals_world
    
    return None

# ==========================================
# PART 2: ROS 2 NAVIGATION
# ==========================================

def main():
    # 1. Get Goals
    goals = get_target_goals()
    if not goals:
        print("Could not calculate path points.")
        return
        
    print(f"\nGenerated {len(goals)} Goals.")
    print(f"1. Far End of Rack 1: ({goals[0][0]:.2f}, {goals[0][1]:.2f})")
    print(f"2. Intersection Point: ({goals[1][0]:.2f}, {goals[1][1]:.2f})")

    # 2. Init ROS
    rclpy.init()
    navigator = BasicNavigator()
    
    print("Waiting for Nav2...")
    if not navigator.lifecycleStartup():
        print("Nav2 lifecycle startup...")

    # 3. Execute Goals Sequentially
    for i, (gx, gy) in enumerate(goals):
        print(f"\n--- Navigating to Goal {i+1} ---")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation.w = 1.0 

        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            # Optional: feedback = navigator.getFeedback()
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Goal {i+1} Reached!")
        else:
            print(f"Goal {i+1} Failed or Canceled.")
            break # Stop if a goal fails
        
    print("Task Finished.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()