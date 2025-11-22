#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import os
import sys
import yaml  # Standard python yaml or pip install pyyaml

# --- CONFIGURATION ---
# Ensure 'maps.yaml' and 'maps.pgm' are in the same directory as this script
YAML_FILENAME = 'new_map_inter_iit.yaml' 
OFFSET_METERS = 1.5        # Distance from wall to path line
START_MARGIN_METERS = 0.5  # 50 cm safety shift from the corner (start point)

# ==========================================
# PART 1: MAP ANALYSIS & GOAL CALCULATION
# ==========================================

def get_map_metadata(yaml_path):
    """
    Reads the map YAML file to get resolution, origin, and image path.
    """
    if not os.path.exists(yaml_path):
        print(f"ERROR: Map YAML not found at {yaml_path}")
        sys.exit(1)

    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
    
    resolution = map_data['resolution']
    origin = map_data['origin'] # [x, y, z]
    image_file = map_data['image']
    
    # Handle relative path for the image based on where the YAML is located
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_path = os.path.join(yaml_dir, image_file)
    
    return resolution, origin, image_path

def get_offset_segment(p1, p2, center_point, offset_dist):
    """
    Calculates a line segment parallel to p1-p2, shifted by offset_dist
    towards the center_point.
    """
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    center = np.array(center_point, dtype=float)
    
    wall_vec = p2 - p1
    ortho_vec = np.array([-wall_vec[1], wall_vec[0]]) 
    norm = np.linalg.norm(ortho_vec)
    
    if norm == 0: 
        return tuple(p1.astype(int)), tuple(p2.astype(int))
    
    ortho_vec = ortho_vec / norm
    
    # Check direction towards center
    midpoint = (p1 + p2) / 2.0
    vec_to_center = center - midpoint
    
    # If dot product is negative, the ortho_vec points OUT. Flip it.
    if np.dot(ortho_vec, vec_to_center) < 0:
        ortho_vec = -ortho_vec
        
    shift = ortho_vec * offset_dist
    return (p1 + shift), (p2 + shift)

def get_line_intersection(line1_p1, line1_p2, line2_p1, line2_p2):
    """
    Finds the intersection (x, y) of two lines.
    """
    x1, y1 = line1_p1
    x2, y2 = line1_p2
    x3, y3 = line2_p1
    x4, y4 = line2_p2
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0: return None # Parallel
    
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
    return (int(px), int(py))

def pixel_to_world(px, py, img_height, res, origin):
    """
    Converts OpenCV pixel coordinates (Top-Left origin) to 
    ROS Map coordinates (Bottom-Left origin).
    """
    wx = (px * res) + origin[0]
    wy = ((img_height - py) * res) + origin[1]
    return (wx, wy)

def get_target_goals():
    """
    Main Logic: Loads map, finds walls, calculates offsets, finding intersection,
    and applies the safety shift to the start point.
    """
    print("--- Analyzing Map ---")
    resolution, origin, image_path = get_map_metadata(YAML_FILENAME)
    
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: Could not load image {image_path}")
        sys.exit(1)

    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
    
    # Find Walls
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("ERROR: No walls found.")
        sys.exit(1)
        
    main_wall_contour = max(contours, key=cv2.contourArea)
    
    # Find Corners & Center
    rect = cv2.minAreaRect(main_wall_contour)
    corners = np.intp(cv2.boxPoints(rect))
    room_center = np.mean(corners, axis=0)
    
    # Find Entrance to determine orientation
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

    # Identify Racks based on Entrance Vector
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
    
    # Sort by distance: [0]=Entrance, [3]=Front(Rack2)
    wall_data.sort(key=lambda x: x['dist'])
    rack2_segment = wall_data[3]['seg'] # Front
    
    # Right Wall logic (Positive Cross Product is Right)
    if wall_data[1]['cp'] > wall_data[2]['cp']:
        rack1_segment = wall_data[1]['seg']
    else:
        rack1_segment = wall_data[2]['seg']

    # Calculate Offsets
    offset_pixels = int(OFFSET_METERS / resolution)
    r1_p1, r1_p2 = get_offset_segment(rack1_segment[0], rack1_segment[1], room_center, offset_pixels)
    r2_p1, r2_p2 = get_offset_segment(rack2_segment[0], rack2_segment[1], room_center, offset_pixels)
    
    # Find Intersection
    intersection_px = get_line_intersection(r1_p1, r1_p2, r2_p1, r2_p2)
    
    goals_world = []

    if intersection_px:
        intersect_arr = np.array(intersection_px)
        
        # 1. Find the "Raw" Far End Corner (furthest from intersection)
        dist1 = np.linalg.norm(np.array(r1_p1) - intersect_arr)
        dist2 = np.linalg.norm(np.array(r1_p2) - intersect_arr)
        
        if dist1 > dist2:
            raw_far_corner = np.array(r1_p1)
        else:
            raw_far_corner = np.array(r1_p2)

        # 2. Calculate Vector: Corner -> Intersection
        vec_to_intersection = intersect_arr - raw_far_corner
        line_length = np.linalg.norm(vec_to_intersection)
        
        # 3. Normalize to get direction
        if line_length > 0:
            direction = vec_to_intersection / line_length
        else:
            direction = np.array([0,0])

        # 4. Apply Safety Margin (Move 10cm inwards along the line)
        shift_pixels = START_MARGIN_METERS / resolution
        
        # Safety check: Don't shift if line is shorter than margin
        if line_length > shift_pixels:
            shifted_start_px = raw_far_corner + (direction * shift_pixels)
        else:
            shifted_start_px = raw_far_corner # Fallback

        final_start_px = tuple(shifted_start_px.astype(int))

        # --- VISUALIZATION (Optional Debug) ---
        debug_img = img.copy()
        cv2.line(debug_img, final_start_px, intersection_px, (255, 0, 255), 2) 
        cv2.circle(debug_img, final_start_px, 8, (0, 165, 255), -1)   # Start (Orange)
        cv2.circle(debug_img, intersection_px, 8, (0, 255, 255), -1)  # End (Yellow)
        cv2.putText(debug_img, "Start (Shifted)", final_start_px, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        cv2.imwrite('debug_goals.png', debug_img)
        print("Debug image saved: 'debug_goals.png'")
        
        # 5. Convert to World Coordinates
        
        # GOAL 1: The Shifted Start Point
        g1_x, g1_y = pixel_to_world(final_start_px[0], final_start_px[1], h, resolution, origin)
        goals_world.append((g1_x, g1_y))
        
        # GOAL 2: The Intersection Point
        g2_x, g2_y = pixel_to_world(intersection_px[0], intersection_px[1], h, resolution, origin)
        goals_world.append((g2_x, g2_y))
        
        return goals_world
    
    return None

# ==========================================
# PART 2: ROS 2 NAVIGATION EXECUTION
# ==========================================

def main():
    # 1. Get Goals
    goals = get_target_goals()
    if not goals:
        print("Could not calculate path points. Check map/debug image.")
        return
        
    print(f"\nGenerated Goals:")
    print(f"1. Rack 1 Start (Shifted {START_MARGIN_METERS*100}cm): X={goals[0][0]:.2f}, Y={goals[0][1]:.2f}")
    print(f"2. Intersection: X={goals[1][0]:.2f}, Y={goals[1][1]:.2f}")

    # 2. Initialize ROS 2
    rclpy.init()
    navigator = BasicNavigator()
    
    print("\nWaiting for Nav2 to be active...")
    # Often useful to just wait for lifecycle if simulation is slow
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
        goal_pose.pose.orientation.w = 1.0  # Standard orientation (Forward-ish)

        navigator.goToPose(goal_pose)

        # Loop while checking status
        while not navigator.isTaskComplete():
            # feedback = navigator.getFeedback()
            # if feedback:
            #     print(f'Distance remaining: {feedback.distance_remaining:.2f}m', end='\r')
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Goal {i+1} Reached!")
        elif result == TaskResult.CANCELED:
            print(f"Goal {i+1} was Canceled!")
            break
        elif result == TaskResult.FAILED:
            print(f"Goal {i+1} Failed!")
            break
        
    print("Task Finished.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()