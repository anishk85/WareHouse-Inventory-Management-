import cv2
import numpy as np
import itertools
import math
import os

# --- Function to find table leg candidates ---
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

    # Map corners
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

    # 3. Find blobs (contours)
    contours, _ = cv2.findContours(
        binary_img,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    leg_points = []
    print(f"\nFound {len(contours)} total dark blobs. Filtering...")

    # 4. Filter the blobs
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


# --- Helper functions for geometry ---
def dist_sq(p1, p2):
    """Calculates the squared Euclidean distance between two points."""
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def order_points(pts):
    """Sorts 4 points into a clockwise order for drawing."""
    cx = np.mean([p[0] for p in pts])
    cy = np.mean([p[1] for p in pts])

    def angle_from_centroid(p):
        return math.atan2(p[1] - cy, p[0] - cx)

    return sorted(pts, key=angle_from_centroid)


# --- Function to find rectangles from 4 points ---
def find_rectangles_from_points(points, output_img, config):
    """
    Finds parallelograms matching dimension constraints from a list of points.

    NOTE: This function only checks for 2 pairs of equal-length sides
    (2 'lengths' and 2 'breadths'). It does NOT check for right angles
    or equal diagonals, so it will find parallelograms, not just rectangles.
    """
    min_L_sq = config['min_len']**2
    max_L_sq = config['max_len']**2
    min_B_sq = config['min_breadth']**2
    max_B_sq = config['max_breadth']**2

    found_rectangles = []

    if len(points) < 4:
        return [], output_img

    # Check every combination of 4 points
    for combo in itertools.combinations(points, 4):
        p1, p2, p3, p4 = combo

        # Calculate all 6 pairs and their distances
        pairs = [
            (p1, p2), (p1, p3), (p1, p4),
            (p2, p3), (p2, p4), (p3, p4)
        ]

        dists_sq = []
        print("\n🔹 Checking 4-point combo:")
        for (a, b) in pairs:
            d2 = dist_sq(a, b)
            d = math.sqrt(d2)
            dists_sq.append(d2)
            print(f"   Pair {a} ↔ {b} → Distance = {d:.2f} px (sq={d2:.2f})")

        # Count matches
        breadth_matches = 0
        length_matches = 0

        for d_sq in dists_sq:
            is_breadth = (min_B_sq <= d_sq <= max_B_sq)
            is_length = (min_L_sq <= d_sq <= max_L_sq)

            if is_breadth:
                breadth_matches += 1
            elif is_length:
                length_matches += 1

        print(f"   Found: {length_matches} length matches, {breadth_matches} breadth matches.")

        # Check if exactly 2 of each (remaining 2 are diagonals)
        if breadth_matches == 2 and length_matches == 2:
            print("✅ 4-Point shape found! (2 matching lengths, 2 matching breadths)")

            # Order points and add to list
            ordered_pts = order_points([p1, p2, p3, p4])
            found_rectangles.append(ordered_pts)

            # Draw the polygon (Yellow)
            pts_array = np.array(ordered_pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(output_img, [pts_array], isClosed=True, color=(0, 255, 255), thickness=2)

    return found_rectangles, output_img


# --- NEW Function to find rectangles from 3 points ---
def find_rectangles_from_3_points(points, output_img, config):
    """
    Finds rectangles by checking 3-point combinations.

    This function checks if 3 points form a right-angled corner
    of the desired rectangle. If so, it infers the 4th point
    and draws the resulting rectangle.
    """
    min_L_sq = config['min_len']**2
    max_L_sq = config['max_len']**2
    min_B_sq = config['min_breadth']**2
    max_B_sq = config['max_breadth']**2
    # Tolerance for (L^2 + B^2 = D^2) check
    pythag_tol_sq = config.get('pythag_tolerance_sq', 50.0)

    found_rectangles = []

    if len(points) < 3:
        return [], output_img

    # Check every combination of 3 points
    for combo in itertools.combinations(points, 3):
        p1, p2, p3 = combo

        # Calculate squared distances
        d12_sq = dist_sq(p1, p2)
        d13_sq = dist_sq(p1, p3)
        d23_sq = dist_sq(p2, p3)

        # Store potential corners: (corner, adj1, adj2, hyp_sq)
        # We test each point as the potential corner of the right angle.
        potentials = [
            (p1, p2, p3, d23_sq),  # p1 is corner, d23 is hypotenuse
            (p2, p1, p3, d13_sq),  # p2 is corner, d13 is hypotenuse
            (p3, p1, p2, d12_sq)   # p3 is corner, d12 is hypotenuse
        ]

        print(f"\n🔹 Checking 3-point combo: {p1}, {p2}, {p3}")

        for corner, adj1, adj2, hyp_sq in potentials:
            side1_sq = dist_sq(corner, adj1)
            side2_sq = dist_sq(corner, adj2)

            # 1. Check for right angle (Pythagorean theorem)
            if abs(side1_sq + side2_sq - hyp_sq) > pythag_tol_sq:
                # print(f"   - {corner} as corner: Fails right-angle check.")
                continue

            # 2. Check if sides match Length and Breadth
            is_L1 = min_L_sq <= side1_sq <= max_L_sq
            is_B1 = min_B_sq <= side1_sq <= max_B_sq
            is_L2 = min_L_sq <= side2_sq <= max_L_sq
            is_B2 = min_B_sq <= side2_sq <= max_B_sq

            # Check for one L and one B (in either order)
            if (is_L1 and is_B2) or (is_B1 and is_L2):
                # Found a valid 3-point corner!
                print(f"✅ 3-Point corner found! Corner: {corner}, Sides: {math.sqrt(side1_sq):.1f}, {math.sqrt(side2_sq):.1f}")

                # Infer the 4th point using vector addition
                # P4 = P_adj1 + P_adj2 - P_corner
                p4_x = adj1[0] + adj2[0] - corner[0]
                p4_y = adj1[1] + adj2[1] - corner[1]
                p4 = (p4_x, p4_y)

                # Order points and add to list
                all_points = [corner, adj1, adj2, p4]
                ordered_pts = order_points(all_points)
                found_rectangles.append(ordered_pts)

                # Draw the inferred point (Blue X)
                cv2.circle(output_img, p4, 8, (255, 100, 0), 2)
                cv2.line(output_img, (p4[0]-5, p4[1]-5), (p4[0]+5, p4[1]+5), (255, 100, 0), 2)
                cv2.line(output_img, (p4[0]-5, p4[1]+5), (p4[0]+5, p4[1]-5), (255, 100, 0), 2)

                # Draw the full polygon (Magenta)
                pts_array = np.array(ordered_pts, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(output_img, [pts_array], isClosed=True, color=(255, 0, 255), thickness=2)

                # Found one, break from checking other corners for this combo
                break

    return found_rectangles, output_img


# --- Main script execution ---
if __name__ == "__main__":
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
        'max_breadth': 27,
        # Tolerance for 3-point check (L^2 + B^2 = D^2)
        # A value of 50 allows for a combined error of ~7 pixels (sqrt(50))
        'pythag_tolerance_sq': 50.0
    }

    try:
        # Determine paths
        curr_dir = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        # Fallback for interactive environments (like Jupyter)
        curr_dir = os.getcwd()
        print("Note: `__file__` not found, using current working directory.")

    image_file = os.path.join(curr_dir, "map.pgm")

    print(f"📂 Script directory: {curr_dir}")
    output_file = os.path.join(curr_dir, "table_legs_detected_with_rectangles2.png")

    try:
        # Stage 1: Find leg candidates
        points, visual_img = find_table_leg_candidates(image_file, blob_config)

        if visual_img is None:
            raise Exception("Image loading failed.")

        print(f"\n--- Found {len(points)} potential table leg candidates ---")
        if points:
            print("Points (x, y):", points)

            # --- Stage 2a: Find rectangles from 4 candidates ---
            print("\n--- STAGE 2a: Checking 4-point combinations (Parallelograms) ---")
            rects_4, visual_img_with_rects = find_rectangles_from_points(
                points,
                visual_img,
                rect_config
            )

            # --- Stage 2b: Find rectangles from 3 candidates (Inferring 4th) ---
            print("\n--- STAGE 2b: Checking 3-point combinations (Right-Angle Rectangles) ---")
            rects_3, visual_img_with_rects = find_rectangles_from_3_points(
                points,
                visual_img_with_rects,  # Use the image from the previous step
                rect_config
            )

            total_rectangles = rects_4 + rects_3

            print("\n\n--- DETECTION SUMMARY ---")
            print(f"Found {len(rects_4)} shapes from 4-point check (Yellow)")
            print(f"Found {len(rects_3)} shapes from 3-point check (Magenta)")
            print(f"--- Total {len(total_rectangles)} matching shapes ---")

            for i, rect in enumerate(total_rectangles):
                print(f"Shape {i+1}: {rect}")

            cv2.imwrite(output_file, visual_img_with_rects)
            print(f"\n✅ Saved visualization with all shapes to {output_file}")
        else:
            print("\nNo leg candidates found, so no shapes to fit.")
            cv2.imwrite(output_file, visual_img)
            print(f"\n✅ Saved visualization (no legs found) to {output_file}")

    except FileNotFoundError:
        print(f"Error: The file '{image_file}' was not found.")
        print("Please ensure 'map.pgm' is in the same directory as the script.")
    except Exception as e:
        print(f"An error occurred: {e}")