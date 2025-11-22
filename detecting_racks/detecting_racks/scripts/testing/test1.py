import cv2
import numpy as np
import itertools
import math

# --- (Your original function - no changes) ---
def find_table_leg_candidates(image_path, config):
    """
    Finds potential table leg locations (small, dark, round-ish blobs)
    in a PGM occupancy map.
    """
    
    # 1. Load the image in grayscale
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print(f"Error: Could not load image from {image_path}")
        return [], None
    
    height, width = img.shape
    print(f"Map dimensions: {width} pixels wide x {height} pixels high")
    
    corners = {
        "Top-Left": (0, 0),
        "Top-Right": (width - 1, 0),
        "Bottom-Left": (0, height - 1),
        "Bottom-Right": (width - 1, height - 1)
    }
    
    print("\n--- Map Pixel Corners ---")
    for name, coord in corners.items():
        print(f"{name}: {coord}")

    output_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    for coord in corners.values():
        cv2.circle(output_img, coord, 10, (255, 0, 0), 2)

    # 2. Thresholding
    _, binary_img = cv2.threshold(
        img, 
        config['occupancy_threshold'], 
        255, 
        cv2.THRESH_BINARY_INV
    )

    # 3. Find Blobs (Contours)
    contours, _ = cv2.findContours(
        binary_img, 
        cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE
    )

    leg_points = []
    print(f"\nFound {len(contours)} total dark blobs. Filtering...")

    # 4. Filter the Blobs
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if config['min_area'] < area < config['max_area']:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            
            if circularity >= config['min_circularity']:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    leg_points.append((cx, cy))
                    cv2.circle(output_img, (cx, cy), 5, (0, 255, 0), -1) 
                    cv2.drawContours(output_img, [cnt], -1, (0, 0, 255), 1)

    return leg_points, output_img

# --- (NEW FUNCTIONS for rectangle fitting) ---

def dist_sq(p1, p2):
    """Calculates the squared Euclidean distance between two points."""
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def order_points(pts):
    """Sorts 4 points into a clockwise order for drawing."""
    # Calculate centroid
    cx = np.mean([p[0] for p in pts])
    cy = np.mean([p[1] for p in pts])
    
    # Sort by angle relative to centroid
    def angle_from_centroid(p):
        return math.atan2(p[1] - cy, p[0] - cx)
        
    return sorted(pts, key=angle_from_centroid)

def find_rectangles_from_points(points, output_img, config):
    """
    Finds rectangles matching dimension constraints from a list of points.
    """
    
    # --- ⚙️ YOU CAN TUNE THESE TOLERANCES ---
    # How much variance (in pixels^2) to allow when comparing side/diagonal lengths
    SIDE_TOLERANCE_SQ = 10.0 
    # How much variance (in pixels^2) to allow for the Pythagorean check
    DIAG_TOLERANCE_SQ = 10.0
    # ----------------------------------------
    
    # Pre-calculate squared dimensions for efficiency
    min_L_sq = config['min_len']**2
    max_L_sq = config['max_len']**2
    min_B_sq = config['min_breadth']**2
    max_B_sq = config['max_breadth']**2

    found_rectangles = []
    
    # 1. Iterate through all combinations of 4 points
    # This is O(n^4), feasible for a small number of candidate points
    if len(points) < 4:
        return [], output_img
        
    for combo in itertools.combinations(points, 4):
        p1, p2, p3, p4 = combo
        
        # 2. Get all 6 squared distances
        dists_sq = [
            dist_sq(p1, p2), dist_sq(p1, p3), dist_sq(p1, p4),
            dist_sq(p2, p3), dist_sq(p2, p4), dist_sq(p3, p4)
        ]
        
        # 3. Sort distances
        dists_sq.sort()
        s1, s2, s3, s4, d1, d2 = dists_sq
        
        # 4. Run checks
        
        # Property 1: Check for a parallelogram (paired sides/diagonals)
        is_parallelogram = (
            abs(s1 - s2) < SIDE_TOLERANCE_SQ and
            abs(s3 - s4) < SIDE_TOLERANCE_SQ and
            abs(d1 - d2) < SIDE_TOLERANCE_SQ
        )
        if not is_parallelogram:
            continue
            
        # Get average lengths for cleaner checks
        avg_S1_sq = (s1 + s2) / 2.0  # Avg squared breadth
        avg_S2_sq = (s3 + s4) / 2.0  # Avg squared length
        avg_D_sq = (d1 + d2) / 2.0  # Avg squared diagonal
        
        # Property 2: Check for rectangle (Pythagorean theorem)
        if abs((avg_S1_sq + avg_S2_sq) - avg_D_sq) > DIAG_TOLERANCE_SQ:
            continue
            
        # Property 3: Check dimensions
        # Check for (Breadth, Length)
        check_A = (
            (min_B_sq <= avg_S1_sq <= max_B_sq) and
            (min_L_sq <= avg_S2_sq <= max_L_sq)
        )
        # Check for (Length, Breadth)
        check_B = (
            (min_L_sq <= avg_S1_sq <= max_L_sq) and
            (min_B_sq <= avg_S2_sq <= max_B_sq)
        )
        
        if check_A or check_B:
            # --- We found a valid rectangle! ---
            
            # Order points for drawing
            ordered_pts = order_points([p1, p2, p3, p4])
            found_rectangles.append(ordered_pts)
            
            # Draw it on the output image
            pts_array = np.array(ordered_pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(output_img, [pts_array], isClosed=True, color=(0, 255, 255), thickness=2)
            
    return found_rectangles, output_img

# --- Main script execution ---
if __name__ == "__main__":
    
    # === ⚙️ YOU MUST TUNE THESE PARAMETERS ===
    blob_config = {
        'occupancy_threshold': 100, 
        'min_area': 0.9,
        'max_area': 10,
        'min_circularity': 0.6 
    }
    
    rect_config = {
        'min_len': 35,
        'max_len': 37,
        'min_breadth': 25,
        'max_breadth': 27
    }
    # =========================================
    
    image_file = "/home/rocinate/zomato_ps/rack_detection_ws/src/turtlebot3/turtlebot3/scripts/map.pgm"
    output_file = "table_legs_detected_with_rectangles.png"

    try:
        # --- Stage 1: Find leg candidates ---
        points, visual_img = find_table_leg_candidates(image_file, blob_config)
        
        if visual_img is None:
            raise Exception("Image loading failed.")

        print(f"\n--- Found {len(points)} potential table leg candidates ---")
        if points:
            print("Points (x, y):", points)

            # --- Stage 2: Find rectangles from those candidates ---
            rectangles, visual_img_with_rects = find_rectangles_from_points(
                points, 
                visual_img, 
                rect_config
            )
            
            print(f"\n--- Found {len(rectangles)} matching rectangles ---")
            for i, rect in enumerate(rectangles):
                print(f"Rectangle {i+1}: {rect}")
                
            # Save the *final* result
            cv2.imwrite(output_file, visual_img_with_rects)
            print(f"\n✅ Saved visualization with rectangles to {output_file}")
            
        else:
            print("\nNo leg candidates found, so no rectangles to fit.")
            # Save the intermediate image anyway
            cv2.imwrite(output_file, visual_img)
            print(f"\n✅ Saved visualization (no legs found) to {output_file}")


    except FileNotFoundError:
        print(f"Error: The file '{image_file}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")