import cv2
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

def find_and_draw_rectangles(image_path, points_list):
    """
    Loads an image, clusters points, and draws the minimum area
    rectangles for each cluster.
    """
    
    # --- 1. Load Image and Prepare Data ---
    
    # Load the original image
    # Make sure 'table_legs_detected.png' is in the same folder
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not load image from {image_path}")
        print("Please make sure the file 'table_legs_detected.png' is in the same directory as the script.")
        return

    # Create a copy to draw on
    img_with_boxes = img.copy()

    # Convert your list of (x, y) points to a NumPy array
    points = np.array(points_list, dtype=np.float32)

    # --- 2. Cluster Points into Four Groups (k=4) ---
    
    # We use K-Means clustering to find the 4 groups (tables)
    # n_init=10 is recommended to avoid bad random starting points
    kmeans = KMeans(n_clusters=4, random_state=0, n_init=10).fit(points)
    labels = kmeans.labels_
    
    # Define colors for the clusters (Blue, Red, Green, Yellow in BGR format)
    colors = [(255, 0, 0), (0, 0, 255), (0, 255, 0), (0, 255, 255)]

    # --- 3. Find and Draw Rectangles for Each Cluster ---
    
    print("Finding bounding boxes for 4 clusters...")
    
    for k in range(4): # Loop for cluster 0, 1, 2, 3
        # Get all points that belong to this cluster
        cluster_points = points[labels == k]

        # We need at least 3 points to define a rectangle
        if len(cluster_points) > 2:
            # Find the minimum area (rotated) rectangle
            # 
            rect = cv2.minAreaRect(cluster_points)
            
            # rect[1] is a tuple (width, height)
            size = rect[1]
            
            # Ensure length is the longer side, breadth is the shorter
            length = max(size)
            breadth = min(size)
            
            # Get the 4 corner points of this rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box) # Convert coordinates to integers

            # Draw the rectangle contour on our image
            cv2.drawContours(img_with_boxes, [box], 0, colors[k], 2) # 2px thickness
            
            print(f"\nCluster {k} Box Corners:")
            print(box)
            
            # --- Updated print statements for dimensions ---
            print(f"Cluster {k} Dimensions (Rotated):")
            print(f"  Length: {length:.2f} pixels")
            print(f"  Breadth: {breadth:.2f} pixels")
            
            # --- Added Axis-Aligned Dimensions ---
            min_x = np.min(cluster_points[:, 0])
            max_x = np.max(cluster_points[:, 0])
            min_y = np.min(cluster_points[:, 1])
            max_y = np.max(cluster_points[:, 1])
            
            axis_width = max_x - min_x
            axis_height = max_y - min_y
            
            print(f"Cluster {k} Dimensions (Axis-Aligned):")
            print(f"  Width (X-axis): {axis_width:.2f} pixels")
            print(f"  Height (Y-axis): {axis_height:.2f} pixels")
            
        else:
            print(f"Cluster {k} had too few points to form a rectangle.")

    # --- (Optional) Draw the original green points ---
    for pt in points_list:
        cv2.circle(img_with_boxes, (int(pt[0]), int(pt[1])), 4, (0, 255, 0), -1) # 4px green circle

    # --- 4. Display the Final Image ---
    
    # OpenCV loads images in BGR format, but Matplotlib displays in RGB.
    # We need to convert the color channels.
    img_rgb = cv2.cvtColor(img_with_boxes, cv2.COLOR_BGR2RGB)

    print("\nDisplaying image with fitted rectangles...")
    plt.figure(figsize=(10, 10))
    plt.imshow(img_rgb)
    plt.title('Fitted Rectangles for Table Legs')
    plt.axis('off') # Hide the x/y axes
    plt.show()


# --- Main execution ---
if __name__ == "__main__":
    
    # The 15 points you provided
    table_leg_points = [
        (98, 280), (82, 259), (126, 257), (194, 241), (110, 237), 
        (177, 221), (205, 198), (120, 193), (231, 184), (103, 173), 
        (148, 170), (214, 164), (258, 161), (130, 150), (241, 141)
    ]
    
    # The path to your original image
    image_file = 'table_legs_detected.png'
    
    find_and_draw_rectangles(image_file, table_leg_points)