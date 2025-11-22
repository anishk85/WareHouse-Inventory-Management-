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
# PART 1: CALCULATE INTERSECTION POINT
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

def get_target_goal():
    print("--- Analyzing Map for Intersection ---")
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
    
    # Calculate Intersection
    off_px = int(OFFSET_METERS / res)
    r1_p1, r1_p2 = get_offset_segment(rack1[0], rack1[1], room_center, off_px)
    r2_p1, r2_p2 = get_offset_segment(rack2[0], rack2[1], room_center, off_px)
    
    intersection_px = get_line_intersection(r1_p1, r1_p2, r2_p1, r2_p2)
    
    if intersection_px:
        # Visualize
        cv2.circle(img, intersection_px, 8, (0, 255, 255), -1)
        cv2.imwrite('debug_intersection.png', img)
        print("Debug image saved: 'debug_intersection.png'")
        
        wx, wy = pixel_to_world(intersection_px[0], intersection_px[1], img.shape[0], res, origin)
        return (wx, wy)
    
    return None

# ==========================================
# PART 2: ROS 2 NAVIGATION
# ==========================================

def main():
    # 1. Get Goal
    target = get_target_goal()
    if not target:
        print("Could not find intersection point.")
        return
        
    tx, ty = target
    print(f"\nTARGET FOUND: X={tx:.3f}, Y={ty:.3f}")

    # 2. Init ROS
    rclpy.init()
    navigator = BasicNavigator()
    
    print("Waiting for Nav2...")
    if not navigator.lifecycleStartup():
        print("Nav2 lifecycle startup...")

    # 3. Send Goal
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = tx
    goal.pose.position.y = ty
    goal.pose.orientation.w = 1.0 # Face forward (standard orientation)

    print(f"Sending Robot to Intersection...")
    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal Reached!")
    else:
        print("Goal Failed/Canceled.")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()