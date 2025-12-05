import cv2
import numpy as np

# --- 1. Load and Pre-process the Image ---

# Load the image in grayscale
image_path = 'edited_inter_iit.pgm'
img = cv2.imread(image_path)
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

# Draw the main wall contour in blue
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

        # Draw a red line across the entrance gap
        if entrance_points:
            cv2.line(output_img, entrance_points[0], entrance_points[1], (0, 0, 255), 3)
            # Calculate the center of the entrance gap
            entrance_center = (
                (entrance_points[0][0] + entrance_points[1][0]) / 2,
                (entrance_points[0][1] + entrance_points[1][1]) / 2
            )

# --- 5. NEW: Identify Racks 1 and 2 ---

rack1_segment = None
rack2_segment = None

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


# --- 6. Draw Racks and Labels ---

# Draw Rack 1 (Orange)
if rack1_segment is not None:
    cv2.line(output_img, rack1_segment[0], rack1_segment[1], (0, 165, 255), 5) # Orange
    mid_x = int((rack1_segment[0][0] + rack1_segment[1][0]) / 2)
    mid_y = int((rack1_segment[0][1] + rack1_segment[1][1]) / 2)
    cv2.putText(output_img, 'Rack 1', (mid_x - 30, mid_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

# Draw Rack 2 (Teal)
if rack2_segment is not None:
    cv2.line(output_img, rack2_segment[0], rack2_segment[1], (0, 100, 255), 5) # Teal
    mid_x = int((rack2_segment[0][0] + rack2_segment[1][0]) / 2)
    mid_y = int((rack2_segment[0][1] + rack2_segment[1][1]) / 2)
    cv2.putText(output_img, 'Rack 2', (mid_x - 30, mid_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

# Draw green circles on all 4 corners (on top)
for corner in corners:
    cv2.circle(output_img, tuple(corner), 7, (0, 255, 0), -1)

# --- 7. Display the Result ---

print(f"Detection Complete:")
print(f" - Main wall detected (blue line).")
print(f" - Found {len(corners)} corners (green dots).")
print(f" - Entrance gap marked (red line).")
if rack1_segment is not None:
    print(f" - Rack 1 marked (orange line).")
if rack2_segment is not None:
    print(f" - Rack 2 marked (teal line).")

# cv2.imshow('Wall Detection', output_img)
cv2.imwrite('map_detected_racks.png', output_img) # Save the result
# cv2.waitKey(0)
cv2.destroyAllWindows()