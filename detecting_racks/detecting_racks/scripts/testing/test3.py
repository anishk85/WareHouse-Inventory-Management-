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


# --- Function to find rectangles from points ---
def find_rectangles_from_points(points, output_img, config):
    """
    Finds quadrilaterals matching dimension constraints from a list of points.

    This version only checks if, among the 6 distances in a 4-point
    combination, exactly 2 match the 'length' config and exactly 2
    match the 'breadth' config.

    It does NOT check for right angles or equal diagonals.
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
        print("\n🔹 Checking combination of points:")
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
            print("✅ Shape found! (2 matching lengths, 2 matching breadths)")

            # Order points and add to list
            ordered_pts = order_points([p1, p2, p3, p4])
            found_rectangles.append(ordered_pts)

            # Draw the polygon
            pts_array = np.array(ordered_pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(output_img, [pts_array], isClosed=True, color=(0, 255, 255), thickness=2)

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
        'max_breadth': 27
    }
    
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    image_file = os.path.join(curr_dir, "map.pgm")

    # Get script directory (where this file is located)
    print(f"📂 Script directory: {curr_dir}")
    output_file = os.path.join(curr_dir, "table_legs_detected_with_rectangles.png")

    try:
        # Stage 1: Find leg candidates
        points, visual_img = find_table_leg_candidates(image_file, blob_config)

        if visual_img is None:
            raise Exception("Image loading failed.")

        print(f"\n--- Found {len(points)} potential table leg candidates ---")
        if points:
            print("Points (x, y):", points)

            # Stage 2: Find rectangles from those candidates
            rectangles, visual_img_with_rects = find_rectangles_from_points(
                points,
                visual_img,
                rect_config
            )

            print(f"\n--- Found {len(rectangles)} matching shapes ---")
            for i, rect in enumerate(rectangles):
                print(f"Shape {i+1}: {rect}")

            cv2.imwrite(output_file, visual_img_with_rects)
            print(f"\n✅ Saved visualization with shapes to {output_file}")
        else:
            print("\nNo leg candidates found, so no shapes to fit.")
            cv2.imwrite(output_file, visual_img)
            print(f"\n✅ Saved visualization (no legs found) to {output_file}")

    except FileNotFoundError:
        print(f"Error: The file '{image_file}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
