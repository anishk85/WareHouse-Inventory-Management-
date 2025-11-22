import cv2
import numpy as np
import os

def find_table_leg_candidates(image_path, config):
    """
    Finds potential table leg locations (small, dark, round-ish blobs)
    in a PGM occupancy map and labels each detected point with its coordinates.
    """
    
    # 1. Load the image in grayscale
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print(f"Error: Could not load image from {image_path}")
        return [], None
    
    height, width = img.shape
    print(f"Map dimensions: {width} pixels wide x {height} pixels high")
    
    # Define corner coordinates
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

    # 2. Thresholding (invert for dark blobs)
    _, binary_img = cv2.threshold(
        img, 
        config['occupancy_threshold'], 
        255, 
        cv2.THRESH_BINARY_INV
    )

    # 3. Find contours
    contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    leg_points = []
    print(f"\nFound {len(contours)} total dark blobs. Filtering...")

    # 4. Filter blobs
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
                    
                    # Draw centroid (green)
                    cv2.circle(output_img, (cx, cy), 5, (0, 255, 0), -1)
                    # Draw contour (red)
                    cv2.drawContours(output_img, [cnt], -1, (0, 0, 255), 1)
                    # Label the point with coordinates
                    label = f"({cx},{cy})"
                    cv2.putText(
                        output_img, 
                        label, 
                        (cx + 8, cy - 8), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.3, 
                        (255, 0, 0),  # BLUE text
                        1, 
                        cv2.LINE_AA
                    )

    return leg_points, output_img


# --- Main script ---
if __name__ == "__main__":
    
    config = {
        'occupancy_threshold': 100, 
        'min_area': 0.1,
        'max_area': 0.5,
        'min_circularity': 0.6 
    }

    # Get script directory (where this file is located)
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"📂 Script directory: {curr_dir}")

    map_file = "maps.pgm"
    if not os.path.exists(os.path.join(curr_dir, map_file)):
        print("❌ No .pgm file found in the script directory.")
        exit(1)

    image_file = os.path.join(curr_dir, map_file)
    output_file = os.path.join(curr_dir, "table_legs_detected.png")

    print(f"🖼 Using input file: {image_file}")
    print(f"💾 Output will be saved as: {output_file}")

    try:
        points, visual_img = find_table_leg_candidates(image_file, config)
        
        if visual_img is not None:
            print(f"\n--- Found {len(points)} potential table leg candidates ---")
            for i, (x, y) in enumerate(points, 1):
                print(f"Leg {i}: ({x}, {y})")
                
            scale_factor = 4
            visual_img = cv2.resize(
                visual_img, 
                None, 
                fx=scale_factor, 
                fy=scale_factor, 
                interpolation=cv2.INTER_NEAREST  # preserves pixel-style maps
)

            cv2.imwrite(output_file, visual_img)
            print(f"\n✅ Saved visualization to {output_file}")

    except Exception as e:
        print(f"⚠️ An error occurred: {e}")
