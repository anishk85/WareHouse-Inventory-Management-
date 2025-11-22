import cv2
import numpy as np

# --- 1. Load and Pre-process the Image ---

image_path = 'maps.pgm'
img = cv2.imread(image_path)

if img is None:
    print(f"Error: Could not load image at {image_path}")
    exit()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Create a binary inverted mask.
_, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

# --- 2. Find the Main Wall Contour ---

contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

try:
    main_wall_contour = max(contours, key=cv2.contourArea)
except ValueError:
    print("Error: No contours found.")
    exit()

output_img = img.copy()
cv2.drawContours(output_img, [main_wall_contour], -1, (255, 0, 0), 2)

# --- 3. Find and Mark 4 Corner Points ---

rect = cv2.minAreaRect(main_wall_contour)
box = cv2.boxPoints(rect)
corners = np.intp(box)

# --- 4. Find the Entrance ---

entrance_points = None
entrance_center = None

if len(main_wall_contour) > 3:
    hull_indices = cv2.convexHull(main_wall_contour, returnPoints=False)
    defects = cv2.convexityDefects(main_wall_contour, hull_indices)

    if defects is not None:
        max_depth = 0
        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            depth = d / 256.0
            if depth > max_depth:
                max_depth = depth
                start_point = tuple(main_wall_contour[s][0])
                end_point = tuple(main_wall_contour[e][0])
                entrance_points = (start_point, end_point)

        if entrance_points:
            cv2.line(output_img, entrance_points[0], entrance_points[1], (0, 0, 255), 3)
            entrance_center = (
                (entrance_points[0][0] + entrance_points[1][0]) / 2,
                (entrance_points[0][1] + entrance_points[1][1]) / 2
            )

# --- 5. NEW: Robust Logic for Rack 1 (Right) and Rack 2 (Front) ---

rack1_segment = None
rack2_segment = None

if entrance_center is not None:
    # 1. Determine the "Heading" Vector (Walking into the room)
    room_center = np.mean(corners, axis=0)
    V_in = room_center - entrance_center  # Vector from Entrance -> Center

    wall_data = []
    
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        
        midpoint = (p1 + p2) / 2.0
        
        # Distance from entrance to this wall's midpoint
        dist_from_entrance = np.linalg.norm(midpoint - entrance_center)
        
        # Vector from Room Center to this Wall's Midpoint
        V_center_to_wall = midpoint - room_center
        
        # Calculate Cross Product to determine Left vs Right
        # In image coordinates (Y is down): 
        # If Cross Product(V_in, V_center_to_wall) > 0, the wall is to the RIGHT.
        # If Cross Product < 0, the wall is to the LEFT.
        cross_prod = np.cross(V_in, V_center_to_wall)
        
        wall_data.append({
            "segment": (tuple(p1), tuple(p2)),
            "distance": dist_from_entrance,
            "cross_product": cross_prod,
            "midpoint": midpoint
        })

    # Sort walls by distance: 
    # [0] = Entrance Wall (Closest)
    # [1], [2] = Side Walls
    # [3] = Front Wall (Farthest)
    wall_data.sort(key=lambda x: x['distance'])

    # --- ASSIGNMENT ---
    
    # Front Wall is the one furthest away
    rack2_segment = wall_data[3]['segment']
    
    # Identify Rack 1 (The Right Wall) between index 1 and 2
    side_wall_A = wall_data[1]
    side_wall_B = wall_data[2]
    
    # We pick the one with the higher (positive) cross product value
    if side_wall_A['cross_product'] > side_wall_B['cross_product']:
        rack1_segment = side_wall_A['segment'] # A is to the Right
    else:
        rack1_segment = side_wall_B['segment'] # B is to the Right

# --- 6. Draw Racks and Labels ---

# Draw Rack 1 (Orange) - THE RIGHT WALL
if rack1_segment is not None:
    cv2.line(output_img, rack1_segment[0], rack1_segment[1], (0, 165, 255), 5)
    mid_x = int((rack1_segment[0][0] + rack1_segment[1][0]) / 2)
    mid_y = int((rack1_segment[0][1] + rack1_segment[1][1]) / 2)
    # Offset text slightly towards room center for visibility
    cv2.putText(output_img, 'Rack 1 (Right)', (mid_x - 40, mid_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

# Draw Rack 2 (Teal) - THE FRONT WALL
if rack2_segment is not None:
    cv2.line(output_img, rack2_segment[0], rack2_segment[1], (0, 100, 255), 5)
    mid_x = int((rack2_segment[0][0] + rack2_segment[1][0]) / 2)
    mid_y = int((rack2_segment[0][1] + rack2_segment[1][1]) / 2)
    cv2.putText(output_img, 'Rack 2 (Front)', (mid_x - 40, mid_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)

# Draw corners
for corner in corners:
    cv2.circle(output_img, tuple(corner), 7, (0, 255, 0), -1)

# --- 7. Display/Save ---
print("Map processed. Rack 1 is Right, Rack 2 is Front.")
cv2.imwrite('map_detected_racks.png', output_img)
# cv2.imshow('Result', output_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()