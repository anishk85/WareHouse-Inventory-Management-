import cv2
import numpy as np

# --- 1. Load and Pre-process the Image ---

# Load the image in grayscale
image_path = 'maps.pgm'
img = cv2.imread(image_path)

if img is None:
    print(f"Error: Could not load image from {image_path}")
    exit()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Create a binary inverted mask.
_, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

# --- 2. Find the Main Wall Contour ---

# Find all contours. RETR_EXTERNAL finds only the outermost boundaries.
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the contour with the largest area
try:
    main_wall_contour = max(contours, key=cv2.contourArea)
except ValueError:
    print("Error: No contours found. Check thresholding settings.")
    exit()

# Create an output image to draw on
output_img = img.copy()
clean_map_img = img.copy() # Create a clean copy for the final PGM output

# Draw the main wall contour in blue (DEBUG ONLY)
cv2.drawContours(output_img, [main_wall_contour], -1, (255, 0, 0), 2)

# --- 3. Find and Mark 4 Corner Points ---

# Get the minimum area bounding rectangle
rect = cv2.minAreaRect(main_wall_contour)
# Get the 4 corner points of the rectangle
box = cv2.boxPoints(rect)
# Convert corners to integers
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

        # Draw a red line across the entrance gap (DEBUG ONLY)
        if entrance_points:
            cv2.line(output_img, entrance_points[0], entrance_points[1], (0, 0, 255), 3)
            # Calculate the center of the entrance gap
            entrance_center = (
                (entrance_points[0][0] + entrance_points[1][0]) / 2,
                (entrance_points[0][1] + entrance_points[1][1]) / 2
            )

# --- 5. Identify Racks 1 and 2 ---

rack1_segment = None
rack2_segment = None
room_center = None

if entrance_center is not None:
    # Find the geometric center of the room
    room_center = np.mean(corners, axis=0)

    # Create an "in" vector (from entrance to room center)
    V_in = room_center - entrance_center
    Vx, Vy = V_in[0], V_in[1]

    # Analyze all 4 wall segments
    wall_data = []
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4] # Use % 4 to wrap around
        
        # Calculate midpoint and distance from entrance
        midpoint = (p1 + p2) / 2.0
        dist = np.linalg.norm(midpoint - entrance_center)
        
        # Calculate wall vector (p1 -> p2)
        V_wall = p2 - p1
        Wx, Wy = V_wall[0], V_wall[1]
        
        # 2D cross product: V_in x V_wall
        # A negative result means V_wall is "clockwise" (right) of V_in
        cross_product_z = Vx * Wy - Vy * Wx
        
        wall_data.append({
            "segment": (tuple(p1), tuple(p2)),
            "distance": dist,
            "cross_product": cross_product_z
        })

    # Sort walls by distance from entrance
    # [0] = closest (entrance wall)
    # [1], [2] = side walls
    # [3] = farthest (Rack 2)
    wall_data.sort(key=lambda w: w['distance'])

    # Rack 2 is the farthest wall
    rack2_segment = wall_data[3]['segment']

    # Rack 1 is the "right" wall (clockwise)
    side_wall_A = wall_data[1]
    side_wall_B = wall_data[2]
    
    # The wall with the negative cross product is on the "right"
    if side_wall_A['cross_product'] < 0:
        rack1_segment = side_wall_A['segment']
    else:
        rack1_segment = side_wall_B['segment']

# --- 6. Compute Parallel Buffer Lines and Intersection ---

def get_midpoint(segment):
    p1 = np.array(segment[0])
    p2 = np.array(segment[1])
    return (p1 + p2) / 2.0

def get_inward_normal(segment, center_point):
    """
    Returns a normalized vector perpendicular to the segment,
    pointing towards the center_point.
    """
    p1 = np.array(segment[0])
    p2 = np.array(segment[1])
    
    # Vector along the wall
    wall_vec = p2 - p1
    dx, dy = wall_vec[0], wall_vec[1]
    
    # Two possible perpendicular vectors: (-dy, dx) and (dy, -dx)
    norm1 = np.array([-dy, dx])
    norm2 = np.array([dy, -dx])
    
    # Normalize them
    if np.linalg.norm(norm1) == 0: return np.array([0,0]) # Safety check
    norm1 = norm1 / np.linalg.norm(norm1)
    norm2 = norm2 / np.linalg.norm(norm2)
    
    # Check which one points closer to the room center
    mid = (p1 + p2) / 2.0
    check_point1 = mid + norm1 * 10
    check_point2 = mid + norm2 * 10
    
    dist1 = np.linalg.norm(check_point1 - center_point)
    dist2 = np.linalg.norm(check_point2 - center_point)
    
    return norm1 if dist1 < dist2 else norm2

def line_intersection(p1, v1, p2, v2):
    """
    Finds intersection of two lines defined by Point + Vector.
    Line 1: p1 + t*v1
    Line 2: p2 + u*v2
    Returns the intersection point as (x, y).
    """
    x1, y1 = p1
    x2, y2 = p1 + v1
    x3, y3 = p2
    x4, y4 = p2 + v2
    
    # Determinant
    denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
    if denom == 0:
        return None  # Parallel lines
        
    px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
    py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom
    return np.array([px, py])

intersection_pt = None
mid_r1 = None
mid_r2 = None
final_r1_start = None
final_r2_start = None

# Configuration
RESOLUTION_M_PX = 0.05  # Meters per pixel
BUFFER_DISTANCE_M = 0.55 # Meters

if rack1_segment is not None and rack2_segment is not None and room_center is not None:
    # 1. Get midpoints and wall vectors
    mid_r1 = get_midpoint(rack1_segment)
    mid_r2 = get_midpoint(rack2_segment)
    
    vec_r1 = np.array(rack1_segment[1]) - np.array(rack1_segment[0])
    vec_r2 = np.array(rack2_segment[1]) - np.array(rack2_segment[0])
    
    # 2. Get inward directions (Normals) for offsetting
    normal_r1 = get_inward_normal(rack1_segment, room_center)
    normal_r2 = get_inward_normal(rack2_segment, room_center)
    
    # 3. Calculate Point on Offset Line (1m inward)
    buffer_pixels = BUFFER_DISTANCE_M / RESOLUTION_M_PX
    offset_pt_r1 = mid_r1 + normal_r1 * buffer_pixels
    offset_pt_r2 = mid_r2 + normal_r2 * buffer_pixels
    
    # 4. Compute Intersection of the two PARALLEL offset lines
    # We use the wall vectors (vec_r1, vec_r2) as the direction,
    # but originating from the offset points.
    intersection_pt = line_intersection(offset_pt_r1, vec_r1, offset_pt_r2, vec_r2)
    
    if intersection_pt is not None:
        # 5. Determine Drawing Segments (Cut off the corner part)
        # For Rack 1: We have the original endpoints. Shift them.
        r1_p1 = np.array(rack1_segment[0]) + normal_r1 * buffer_pixels
        r1_p2 = np.array(rack1_segment[1]) + normal_r1 * buffer_pixels
        
        # Identify which endpoint is FARTHER from the OTHER rack (Rack 2's midpoint)
        # This effectively keeps the main length and cuts the corner overlap.
        d1 = np.linalg.norm(r1_p1 - mid_r2)
        d2 = np.linalg.norm(r1_p2 - mid_r2)
        final_r1_start = r1_p1 if d1 > d2 else r1_p2
        
        # For Rack 2: Shift endpoints.
        r2_p1 = np.array(rack2_segment[0]) + normal_r2 * buffer_pixels
        r2_p2 = np.array(rack2_segment[1]) + normal_r2 * buffer_pixels
        
        # Identify endpoint farther from Rack 1's midpoint
        d1 = np.linalg.norm(r2_p1 - mid_r1)
        d2 = np.linalg.norm(r2_p2 - mid_r1)
        final_r2_start = r2_p1 if d1 > d2 else r2_p2

# --- 7. Draw Everything ---

# Draw Rack 1 (Orange) on DEBUG image
if rack1_segment is not None:
    cv2.line(output_img, rack1_segment[0], rack1_segment[1], (0, 165, 255), 5) # Orange
    mid_x = int((rack1_segment[0][0] + rack1_segment[1][0]) / 2)
    mid_y = int((rack1_segment[0][1] + rack1_segment[1][1]) / 2)
    cv2.putText(output_img, 'Rack 1', (mid_x - 30, mid_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

# Draw Rack 2 (Teal) on DEBUG image
if rack2_segment is not None:
    cv2.line(output_img, rack2_segment[0], rack2_segment[1], (0, 100, 255), 5) # Teal
    mid_x = int((rack2_segment[0][0] + rack2_segment[1][0]) / 2)
    mid_y = int((rack2_segment[0][1] + rack2_segment[1][1]) / 2)
    cv2.putText(output_img, 'Rack 2', (mid_x - 30, mid_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

# Draw Buffer Lines
if intersection_pt is not None and final_r1_start is not None:
    ix, iy = int(intersection_pt[0]), int(intersection_pt[1])
    s1x, s1y = int(final_r1_start[0]), int(final_r1_start[1])
    s2x, s2y = int(final_r2_start[0]), int(final_r2_start[1])
    
    # --- Draw on DEBUG image (Purple with marker) ---
    cv2.line(output_img, (s1x, s1y), (ix, iy), (255, 0, 255), 2)
    cv2.line(output_img, (s2x, s2y), (ix, iy), (255, 0, 255), 2)
    cv2.circle(output_img, (ix, iy), 5, (255, 0, 255), -1)

    # --- Draw on CLEAN MAP image (Black lines only) ---
    # Using Black (0, 0, 0) so they appear as obstacles in the PGM map
    cv2.line(clean_map_img, (s1x, s1y), (ix, iy), (0, 0, 0), 2)
    cv2.line(clean_map_img, (s2x, s2y), (ix, iy), (0, 0, 0), 2)

# Draw green circles on all 4 corners (DEBUG ONLY)
for corner in corners:
    cv2.circle(output_img, tuple(corner), 7, (0, 255, 0), -1)

# --- 8. Display the Result ---

print(f"Detection Complete:")
print(f" - Main wall detected (blue line).")
print(f" - Found {len(corners)} corners (green dots).")
print(f" - Entrance gap marked (red line).")

if rack1_segment is not None:
    print(f" - Rack 1 marked (orange line).")
if rack2_segment is not None:
    print(f" - Rack 2 marked (teal line).")
if intersection_pt is not None:
    print(f" - Parallel Buffer lines drawn intersecting at: {intersection_pt}")
    print(f" - Buffer Distance: {BUFFER_DISTANCE_M}m ({BUFFER_DISTANCE_M / RESOLUTION_M_PX:.1f} pixels)")

# Save the CLEAN PGM/PNG (Only original map + black buffer lines)
# PGM requires grayscale. Convert BGR output to Grayscale before saving.
output_gray_final = cv2.cvtColor(clean_map_img, cv2.COLOR_BGR2GRAY)
cv2.imwrite('new_maps.pgm', output_gray_final)
cv2.imwrite('new_maps.png', output_gray_final)

# Save the DEBUG color image (with all text, colors, corners)
cv2.imwrite('map_with_buffers.png', output_img)

print("Result saved as 'new_maps.pgm', 'new_maps.png' (grayscale, clean) and 'map_with_buffers.png' (color, debug).")
# cv2.destroyAllWindows()