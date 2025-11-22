import cv2
import numpy as np

# --- Constants ---
RESOLUTION = 0.05  # meters per pixel
OFFSET_METERS = 1.0
OFFSET_PIXELS = int(OFFSET_METERS / RESOLUTION)

# --- Helper: Calculate Offset Segment ---
def get_offset_segment(p1, p2, center_point, offset_dist):
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    center = np.array(center_point, dtype=float)
    
    wall_vec = p2 - p1
    ortho_vec = np.array([-wall_vec[1], wall_vec[0]]) # Perpendicular
    
    norm = np.linalg.norm(ortho_vec)
    if norm == 0: return tuple(p1.astype(int)), tuple(p2.astype(int))
    ortho_vec = ortho_vec / norm
    
    # Check direction towards center
    midpoint = (p1 + p2) / 2.0
    vec_to_center = center - midpoint
    if np.dot(ortho_vec, vec_to_center) < 0:
        ortho_vec = -ortho_vec
        
    shift = ortho_vec * offset_dist
    return (p1 + shift), (p2 + shift) # Return as floats for precision intersection

# --- Helper: Find Line Intersection ---
def get_line_intersection(line1_p1, line1_p2, line2_p1, line2_p2):
    """
    Finds the intersection (x, y) of two lines defined by (p1, p2) and (p3, p4).
    Uses determinants.
    """
    x1, y1 = line1_p1
    x2, y2 = line1_p2
    x3, y3 = line2_p1
    x4, y4 = line2_p2
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None # Parallel lines
    
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
    
    return (int(px), int(py))

# --- Main Process ---

image_path = 'maps.pgm'
img = cv2.imread(image_path)

if img is None:
    print("Error loading image.")
    exit()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

# 1. Find Walls
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
main_wall_contour = max(contours, key=cv2.contourArea)

output_img = img.copy()
cv2.drawContours(output_img, [main_wall_contour], -1, (255, 0, 0), 2)

# 2. Find Corners & Center
rect = cv2.minAreaRect(main_wall_contour)
box = cv2.boxPoints(rect)
corners = np.intp(box)
room_center = np.mean(corners, axis=0)

# 3. Find Entrance
entrance_center = None
entrance_points = None
# (Simplified entrance logic for brevity, identical to previous steps)
if len(main_wall_contour) > 3:
    hull_indices = cv2.convexHull(main_wall_contour, returnPoints=False)
    defects = cv2.convexityDefects(main_wall_contour, hull_indices)
    if defects is not None:
        max_depth = 0
        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            if (d/256.0) > max_depth:
                max_depth = d/256.0
                entrance_points = (tuple(main_wall_contour[s][0]), tuple(main_wall_contour[e][0]))
        if entrance_points:
            cv2.line(output_img, entrance_points[0], entrance_points[1], (0, 0, 255), 3)
            entrance_center = ((entrance_points[0][0]+entrance_points[1][0])/2, 
                               (entrance_points[0][1]+entrance_points[1][1])/2)

# 4. Identify Racks
rack1_segment = None # Right
rack2_segment = None # Front

if entrance_center is not None:
    V_in = room_center - entrance_center
    wall_data = []
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        mid = (p1 + p2) / 2.0
        dist = np.linalg.norm(mid - entrance_center)
        cp = np.cross(V_in, mid - room_center)
        wall_data.append({'seg': (p1, p2), 'dist': dist, 'cp': cp})

    wall_data.sort(key=lambda x: x['dist'])
    rack2_segment = wall_data[3]['seg'] # Farthest
    # Determine Right Wall
    if wall_data[1]['cp'] > wall_data[2]['cp']:
        rack1_segment = wall_data[1]['seg']
    else:
        rack1_segment = wall_data[2]['seg']

# --- 5. Calculate and Draw Joined Offsets ---

if rack1_segment is not None and rack2_segment is not None:
    # Draw Original Walls
    cv2.line(output_img, tuple(map(int, rack1_segment[0])), tuple(map(int, rack1_segment[1])), (0, 165, 255), 5)
    cv2.line(output_img, tuple(map(int, rack2_segment[0])), tuple(map(int, rack2_segment[1])), (0, 100, 255), 5)

    # 1. Get Raw Offset Lines (Float coordinates)
    r1_off_p1, r1_off_p2 = get_offset_segment(rack1_segment[0], rack1_segment[1], room_center, OFFSET_PIXELS)
    r2_off_p1, r2_off_p2 = get_offset_segment(rack2_segment[0], rack2_segment[1], room_center, OFFSET_PIXELS)

    # 2. Find Intersection Point
    intersection = get_line_intersection(r1_off_p1, r1_off_p2, r2_off_p1, r2_off_p2)

    if intersection:
        # 3. Draw Rack 1 Offset (Right Wall)
        # We want to draw from the point FURTHEST from the intersection, TO the intersection.
        dist_p1 = np.linalg.norm(r1_off_p1 - intersection)
        dist_p2 = np.linalg.norm(r1_off_p2 - intersection)
        start_point_r1 = tuple(r1_off_p1.astype(int)) if dist_p1 > dist_p2 else tuple(r1_off_p2.astype(int))
        
        cv2.line(output_img, start_point_r1, intersection, (255, 0, 255), 2)

        # 4. Draw Rack 2 Offset (Front Wall)
        # Same logic: keep the outer point, connect to intersection
        dist_p1 = np.linalg.norm(r2_off_p1 - intersection)
        dist_p2 = np.linalg.norm(r2_off_p2 - intersection)
        start_point_r2 = tuple(r2_off_p1.astype(int)) if dist_p1 > dist_p2 else tuple(r2_off_p2.astype(int))
        
        cv2.line(output_img, start_point_r2, intersection, (255, 0, 255), 2)
        
        # Mark the join point
        cv2.circle(output_img, intersection, 5, (255, 255, 0), -1)

    # Labels
    mx, my = np.mean(rack1_segment, axis=0).astype(int)
    cv2.putText(output_img, 'Rack 1', (mx-40, my), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
    mx, my = np.mean(rack2_segment, axis=0).astype(int)
    cv2.putText(output_img, 'Rack 2', (mx-40, my), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)

# Draw Green Corners
for c in corners:
    cv2.circle(output_img, tuple(c), 7, (0, 255, 0), -1)

print("Processing Complete. Offset lines joined at intersection.")
cv2.imwrite('wall_offset_joined.png', output_img)
# cv2.imshow('Result', output_img)
# cv2.waitKey(0)
cv2.destroyAllWindows()