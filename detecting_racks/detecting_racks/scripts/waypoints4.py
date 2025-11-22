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
import math  # Added for atan2

# --- CONFIGURATION ---
YAML_FILENAME = 'new_map_inter_iit.yaml' 
OFFSET_METERS = 1.5        
START_MARGIN_METERS = 0.5  
ITERATIONS = 5  # Number of "To and Fro" cycles

# ==========================================
# PART 1: MAP ANALYSIS & GOAL CALCULATION
# ==========================================

def get_map_metadata(yaml_path):
    if not os.path.exists(yaml_path):
        print(f"ERROR: Map YAML not found at {yaml_path}")
        sys.exit(1)

    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
    
    resolution = map_data['resolution']
    origin = map_data['origin'] 
    image_file = map_data['image']
    
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_path = os.path.join(yaml_dir, image_file)
    
    return resolution, origin, image_path

def get_offset_segment(p1, p2, center_point, offset_dist):
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    center = np.array(center_point, dtype=float)
    
    wall_vec = p2 - p1
    ortho_vec = np.array([-wall_vec[1], wall_vec[0]]) 
    norm = np.linalg.norm(ortho_vec)
    
    if norm == 0: 
        return tuple(p1.astype(int)), tuple(p2.astype(int))
    
    ortho_vec = ortho_vec / norm
    
    midpoint = (p1 + p2) / 2.0
    vec_to_center = center - midpoint
    
    if np.dot(ortho_vec, vec_to_center) < 0:
        ortho_vec = -ortho_vec
        
    shift = ortho_vec * offset_dist
    return (p1 + shift), (p2 + shift)

def get_line_intersection(line1_p1, line1_p2, line2_p1, line2_p2):
    x1, y1 = line1_p1
    x2, y2 = line1_p2
    x3, y3 = line2_p1
    x4, y4 = line2_p2
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0: return None 
    
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
    return (int(px), int(py))

def pixel_to_world(px, py, img_height, res, origin):
    wx = (px * res) + origin[0]
    wy = ((img_height - py) * res) + origin[1]
    return (wx, wy)

def get_target_goals():
    print("--- Analyzing Map ---")
    resolution, origin, image_path = get_map_metadata(YAML_FILENAME)
    
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: Could not load image {image_path}")
        sys.exit(1)

    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("ERROR: No walls found.")
        sys.exit(1)
        
    main_wall_contour = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(main_wall_contour)
    corners = np.intp(cv2.boxPoints(rect))
    room_center = np.mean(corners, axis=0)
    
    entrance_center = None
    if len(main_wall_contour) > 3:
        hull = cv2.convexHull(main_wall_contour, returnPoints=False)
        defects = cv2.convexityDefects(main_wall_contour, hull)
        if defects is not None:
            max_depth = 0
            best_pts = None
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                if (d/256.0) > max_depth:
                    max_depth = d/256.0
                    best_pts = (main_wall_contour[s][0], main_wall_contour[e][0])
            if best_pts is not None:
                entrance_center = np.mean(best_pts, axis=0)

    if entrance_center is None:
        print("ERROR: Could not detect entrance.")
        sys.exit(1)

    V_in = room_center - entrance_center
    wall_data = []
    for i in range(4):
        p1, p2 = corners[i], corners[(i+1)%4]
        mid = (p1+p2)/2.0
        wall_data.append({
            'seg': (p1, p2), 
            'dist': np.linalg.norm(mid - entrance_center), 
            'cp': np.cross(V_in, mid - room_center)
        })
    
    wall_data.sort(key=lambda x: x['dist'])
    rack2_segment = wall_data[3]['seg'] 
    
    if wall_data[1]['cp'] > wall_data[2]['cp']:
        rack1_segment = wall_data[1]['seg']
    else:
        rack1_segment = wall_data[2]['seg']

    offset_pixels = int(OFFSET_METERS / resolution)
    r1_p1, r1_p2 = get_offset_segment(rack1_segment[0], rack1_segment[1], room_center, offset_pixels)
    r2_p1, r2_p2 = get_offset_segment(rack2_segment[0], rack2_segment[1], room_center, offset_pixels)
    
    intersection_px = get_line_intersection(r1_p1, r1_p2, r2_p1, r2_p2)
    
    goals_world = []

    if intersection_px:
        intersect_arr = np.array(intersection_px)
        dist1 = np.linalg.norm(np.array(r1_p1) - intersect_arr)
        dist2 = np.linalg.norm(np.array(r1_p2) - intersect_arr)
        
        if dist1 > dist2:
            raw_far_corner = np.array(r1_p1)
        else:
            raw_far_corner = np.array(r1_p2)

        vec_to_intersection = intersect_arr - raw_far_corner
        line_length = np.linalg.norm(vec_to_intersection)
        
        if line_length > 0:
            direction = vec_to_intersection / line_length
        else:
            direction = np.array([0,0])

        shift_pixels = START_MARGIN_METERS / resolution
        
        if line_length > shift_pixels:
            shifted_start_px = raw_far_corner + (direction * shift_pixels)
        else:
            shifted_start_px = raw_far_corner

        final_start_px = tuple(shifted_start_px.astype(int))

        g1_x, g1_y = pixel_to_world(final_start_px[0], final_start_px[1], h, resolution, origin)
        goals_world.append((g1_x, g1_y))
        
        g2_x, g2_y = pixel_to_world(intersection_px[0], intersection_px[1], h, resolution, origin)
        goals_world.append((g2_x, g2_y))
        
        return goals_world
    
    return None

# ==========================================
# PART 2: ROS 2 NAVIGATION EXECUTION
# ==========================================

def get_quaternion_from_yaw(yaw):
    """
    Convert a yaw angle (radians) to a Quaternion message (x,y,z,w)
    """
    q_x = 0.0
    q_y = 0.0
    q_z = math.sin(yaw / 2.0)
    q_w = math.cos(yaw / 2.0)
    return q_x, q_y, q_z, q_w

def get_orientation_between_points(p_start, p_end):
    """
    Calculates yaw between two points (x,y) and returns quaternion
    """
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    yaw = math.atan2(dy, dx)
    return get_quaternion_from_yaw(yaw)

def move_to_pose(navigator, x, y, q_z, q_w, desc="Target"):
    """
    Helper wrapper to send a goal and wait for result
    """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w

    print(f"Going to {desc} (X:{x:.2f}, Y:{y:.2f})...")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"{desc} Reached!")
        return True
    else:
        print(f"{desc} Failed or Canceled!")
        return False

def main():
    # 1. Get Goals
    goals = get_target_goals()
    if not goals:
        print("Could not calculate path points.")
        return
    
    # Unpack points
    start_pt = goals[0] # Point A (Far Corner)
    end_pt = goals[1]   # Point B (Intersection)

    # Calculate Orientations
    # Q_forward: Facing B from A
    qx1, qy1, qz1, qw1 = get_orientation_between_points(start_pt, end_pt)
    # Q_backward: Facing A from B (180 deg flip)
    qx2, qy2, qz2, qw2 = get_orientation_between_points(end_pt, start_pt)

    # 2. Initialize ROS 2
    rclpy.init()
    navigator = BasicNavigator()
    
    print("\nWaiting for Nav2 to be active...")
    navigator.lifecycleStartup()

    # 3. Execution Phase
    
    # Step A: Go to Start Point first (Initial setup)
    print("\n--- Positioning at Start Point ---")
    if not move_to_pose(navigator, start_pt[0], start_pt[1], qz1, qw1, "Start Point"):
        rclpy.shutdown()
        return

    # Step B: Loop 5 times (To and Fro)
    print(f"\n--- Starting {ITERATIONS} To-and-Fro Iterations ---")
    
    for i in range(ITERATIONS):
        print(f"\n>>> ITERATION {i+1} / {ITERATIONS}")
        
        # 1. Move Forward (Start -> Intersection)
        success = move_to_pose(navigator, end_pt[0], end_pt[1], qz1, qw1, "Intersection (Forward)")
        if not success: break
        
        # 2. Move Backward (Intersection -> Start)
        success = move_to_pose(navigator, start_pt[0], start_pt[1], qz2, qw2, "Start Point (Backward)")
        if not success: break

    print("Task Finished.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()