import cv2
import numpy as np

# --- Constants ---
RESOLUTION = 0.05  # meters per pixel
OFFSET_METERS = 1.0
OFFSET_PIXELS = int(OFFSET_METERS / RESOLUTION)  # 20 pixels

# --- Helper Function to Calculate Offset Line ---
def get_offset_segment(p1, p2, center_point, offset_dist):
    """
    Calculates a line segment parallel to p1-p2, shifted by offset_dist
    towards the center_point.
    """
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    center = np.array(center_point, dtype=float)

    # 1. Calculate Wall Vector
    wall_vec = p2 - p1
    
    # 2. Calculate Perpendicular Vector (-y, x)
    ortho_vec = np.array([-wall_vec[1], wall_vec[0]])
    
    # 3. Normalize the perpendicular vector
    norm = np.linalg.norm(ortho_vec)
    if norm == 0: return tuple(p1.astype(int)), tuple(p2.astype(int))
    ortho_vec = ortho_vec / norm
    
    # 4. Determine direction: Dot product with vector to center
    # Midpoint of the wall
    midpoint = (p1 + p2) / 2.0
    vec_to_center = center - midpoint
    
    # If dot product is negative, the ortho_vec points OUT. Flip it.
    if np.dot(ortho_vec, vec_to_center) < 0:
        ortho_vec = -ortho_vec
        
    # 5. Apply Offset
    shift = ortho_vec * offset_dist
    new_p1 = p1 + shift
    new_p2 = p2 + shift
    
    return tuple(new_p1.astype(int)), tuple(new_p2.astype(int))

# --- 1. Load and Pre-process ---

image_path = 'maps.pgm'
img = cv2.imread(image_path)

if img is None:
    print(f"Error: Could not load image at {image_path}")
    exit()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

# --- 2. Find Main Wall Contour ---

contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
try:
    main_wall_contour = max(contours, key=cv2.contourArea)
except ValueError:
    exit()

output_img = img.copy()
# Draw original wall in Blue
cv2.drawContours(output_img, [main_wall_contour], -1, (255, 0, 0), 2)

# --- 3. Find Corners ---

rect = cv2.minAreaRect(main_wall_contour)
box = cv2.boxPoints(rect)
corners = np.intp(box)
room_center = np.mean(corners, axis=0) # Calculate room center here for use later

# --- 4. Find Entrance ---

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

# --- 5. Identify Racks (Right and Front) ---

rack1_segment = None
rack2_segment = None

if entrance_center is not None:
    V_in = room_center - entrance_center

    wall_data = []
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        midpoint = (p1 + p2) / 2.0
        dist = np.linalg.norm(midpoint - entrance_center)
        
        V_center_to_wall = midpoint - room_center
        cross_prod = np.cross(V_in, V_center_to_wall)
        
        wall_data.append({
            "segment": (tuple(p1), tuple(p2)),
            "distance": dist,
            "cross_product": cross_prod
        })

    wall_data.sort(key=lambda x: x['distance'])

    # Rack 2 is the Front wall (Farthest)
    rack2_segment = wall_data[3]['segment']
    
    # Rack 1 is the Right Wall (Higher Cross Product)
    if wall_data[1]['cross_product'] > wall_data[2]['cross_product']:
        rack1_segment = wall_data[1]['segment']
    else:
        rack1_segment = wall_data[2]['segment']

# --- 6. Draw Racks and Offsets ---

# --- RACK 1 (RIGHT) ---
if rack1_segment is not None:
    # Draw Wall (Orange)
    cv2.line(output_img, rack1_segment[0], rack1_segment[1], (0, 165, 255), 5)
    
    # Calculate Offset Line
    off_p1, off_p2 = get_offset_segment(rack1_segment[0], rack1_segment[1], room_center, OFFSET_PIXELS)
    
    # Draw Offset Line (Purple)
    cv2.line(output_img, off_p1, off_p2, (255, 0, 255), 2)
    
    # Label
    mid_x = int((rack1_segment[0][0] + rack1_segment[1][0]) / 2)
    mid_y = int((rack1_segment[0][1] + rack1_segment[1][1]) / 2)
    cv2.putText(output_img, 'Rack 1', (mid_x - 40, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

# --- RACK 2 (FRONT) ---
if rack2_segment is not None:
    # Draw Wall (Teal)
    cv2.line(output_img, rack2_segment[0], rack2_segment[1], (0, 100, 255), 5)
    
    # Calculate Offset Line
    off_p1, off_p2 = get_offset_segment(rack2_segment[0], rack2_segment[1], room_center, OFFSET_PIXELS)
    
    # Draw Offset Line (Purple)
    cv2.line(output_img, off_p1, off_p2, (255, 0, 255), 2)

    # Label
    mid_x = int((rack2_segment[0][0] + rack2_segment[1][0]) / 2)
    mid_y = int((rack2_segment[0][1] + rack2_segment[1][1]) / 2)
    cv2.putText(output_img, 'Rack 2', (mid_x - 40, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)

# Draw corners
for corner in corners:
    cv2.circle(output_img, tuple(corner), 7, (0, 255, 0), -1)

# --- 7. Output ---
print(f"Processing Complete.")
print(f"Resolution: {RESOLUTION} m/px")
print(f"Offset: {OFFSET_METERS}m = {OFFSET_PIXELS} pixels")
print("Purple lines indicate the 1m inner offset.")

cv2.imwrite('map_detected_with_offset.png', output_img)
# cv2.imshow('Result', output_img)
# cv2.waitKey(0)
cv2.destroyAllWindows()