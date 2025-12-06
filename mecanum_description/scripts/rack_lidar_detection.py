#!/usr/bin/env python3

"""
ROS 2 Node for Table Detection in Occupancy Grids.

This node loads a PGM occupancy grid map, scans it for small, circular
blobs (assumed to be table legs), and then attempts to find 3-point and
4-point combinations of these blobs that match the configured dimensions
of a rectangular table.

Detected tables are published as a visualization_msgs/MarkerArray
on the /detected_tables topic.

This node is intended to be run as a one-shot script. It will load the map,
publish the markers, and then shut down.
"""

import os
import math
import time
import itertools
import traceback
from typing import List, Tuple, Dict, Any, Optional, Set

# --- Third-Party Imports ---
import cv2
import numpy as np

# --- ROS 2 Imports ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# --- Type Aliases for Clarity ---
PointTuple = Tuple[int, int]
"""A type alias for a pixel coordinate (x, y)."""

RectPoints = List[PointTuple]
"""A type alias for a list of 4 PointTuples defining a rectangle."""


class TableDetectorNode(Node):
    """
    Detects rectangular tables from a static PGM map file.

    This node works by:
    1. Receiving configuration dictionaries (map, blob, rect) on init.
    2. Getting the 'map_file_path' parameter.
    3. Loading the PGM map image.
    4. Finding blob candidates (table legs) using OpenCV.
    5. Finding 4-point and 3-point rectangle combinations.
    6. Publishing the results as a MarkerArray.
    """

    def __init__(
        self,
        map_config: Dict[str, Any],
        blob_config: Dict[str, Any],
        rect_config: Dict[str, Any],
    ):
        """
        Initializes the node and stores configurations.

        Args:
            map_config: Dictionary with map resolution and origin.
            blob_config: Dictionary with blob detection parameters.
            rect_config: Dictionary with rectangle dimension parameters.
        """
        super().__init__('table_detector')

        # --- Store configurations passed from main ---
        self.map_config = map_config
        self.blob_config = blob_config
        self.rect_config = rect_config

        # --- Declare the one parameter we want to be overridable ---
        # Default 'map.pgm' assumes it's in the directory where you run ros2 run
        self.declare_parameter('map_file_path', 'map.pgm')

        # --- Create Publisher ---
        self.marker_pub = self.create_publisher(
            MarkerArray, '/detected_tables', 10
        )

        self.get_logger().info("ROS 2 Table Detector node initialized.")
        self.get_logger().info("Publishing markers to /detected_tables")

    def _log_configuration(self):
        """Prints the loaded configuration to the ROS logger."""
        self.get_logger().info("\n--- Map & Object Configuration ---")
        self.get_logger().info(
            f"Map Resolution: {self.map_config['resolution']} m/pixel"
        )
        self.get_logger().info(
            f"Map Origin (x,y): ({self.map_config['origin_x']}, "
            f"{self.map_config['origin_y']}) m"
        )

        res = self.map_config['resolution']
        min_l_px = self.rect_config['min_len_m'] / res
        max_l_px = self.rect_config['max_len_m'] / res
        min_b_px = self.rect_config['min_breadth_m'] / res
        max_b_px = self.rect_config['max_breadth_m'] / res

        self.get_logger().info(
            f"Target Length: {self.rect_config['min_len_m']}-"
            f"{self.rect_config['max_len_m']} m  "
            f"->  ({min_l_px:.1f}-{max_l_px:.1f} pixels)"
        )
        self.get_logger().info(
            f"Target Breadth: {self.rect_config['min_breadth_m']}-"
            f"{self.rect_config['max_breadth_m']} m "
            f"->  ({min_b_px:.1f}-{max_b_px:.1f} pixels)"
        )

    # --- Static Helper Functions ---
    @staticmethod
    def _dist_sq(p1: PointTuple, p2: PointTuple) -> float:
        """Calculates the squared Euclidean distance between two points."""
        return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2

    @staticmethod
    def _order_points(pts: List[PointTuple]) -> RectPoints:
        """Sorts 4 points into a clockwise order for drawing."""
        cx = np.mean([p[0] for p in pts])
        cy = np.mean([p[1] for p in pts])

        def angle_from_centroid(p):
            return math.atan2(p[1] - cy, p[0] - cx)

        return sorted(pts, key=angle_from_centroid)

    # --- Core Detection Logic Methods ---

    def _find_table_leg_candidates(
        self, image_path: str
    ) -> Tuple[List[PointTuple], Optional[np.ndarray]]:
        """
        Finds potential table leg locations in a PGM occupancy map.
        Uses the `self.blob_config` dictionary for parameters.

        Args:
            image_path: The filesystem path to the PGM image.

        Returns:
            A tuple containing:
            - A list of (x, y) pixel coordinates for candidates.
            - The annotated output image (or None if loading fails).
        """
        # 1. Load the image in grayscale
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Could not load image from {image_path}")
            return [], None

        height, width = img.shape
        self.get_logger().info(
            f"Map dimensions: {width} pixels wide x {height} pixels high"
        )

        # Store map height in config for coordinate transforms
        self.map_config['map_height_pixels'] = height

        output_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # 2. Thresholding
        _, binary_img = cv2.threshold(
            img,
            self.blob_config['occupancy_threshold'],
            255,
            cv2.THRESH_BINARY_INV,
        )

        # 3. Find blobs (contours)
        contours, _ = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        leg_points: List[PointTuple] = []
        # 4. Filter the blobs
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if (
                self.blob_config['min_area']
                < area
                < self.blob_config['max_area']
            ):
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = (4 * np.pi * area) / (perimeter * perimeter)

                if circularity >= self.blob_config['min_circularity']:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        leg_points.append((cx, cy))

        return leg_points, output_img

    def _find_rectangles_from_points(
        self,
        points: List[PointTuple],
        output_img: np.ndarray,
        id_counter: int = 0,
    ) -> Tuple[List[RectPoints], np.ndarray]:
        """
        Finds parallelograms matching dimension constraints from 4 points.
        Uses `self.rect_config` and `self.map_config`.

        Args:
            points: List of candidate (x, y) leg points.
            output_img: The OpenCV image to draw visualizations on.
            id_counter: The starting ID number for new rectangles.

        Returns:
            A tuple containing:
            - A list of found rectangles [RectPoints, ...].
            - The annotated output image.
        """
        # Convert real-world meters to squared pixel distances
        res = self.map_config['resolution']
        min_L_sq = (self.rect_config['min_len_m'] / res) ** 2
        max_L_sq = (self.rect_config['max_len_m'] / res) ** 2
        min_B_sq = (self.rect_config['min_breadth_m'] / res) ** 2
        max_B_sq = (self.rect_config['max_breadth_m'] / res) ** 2

        found_rectangles: List[RectPoints] = []

        if len(points) < 4:
            return [], output_img

        # Check every combination of 4 points
        for combo in itertools.combinations(points, 4):
            p1, p2, p3, p4 = combo

            # Calculate all 6 pairs and their distances
            pairs = [
                (p1, p2), (p1, p3), (p1, p4),
                (p2, p3), (p2, p4), (p3, p4),
            ]
            dists_sq = [self._dist_sq(a, b) for (a, b) in pairs]

            # Count matches
            breadth_matches = 0
            length_matches = 0

            for d_sq in dists_sq:
                if min_B_sq <= d_sq <= max_B_sq:
                    breadth_matches += 1
                elif min_L_sq <= d_sq <= max_L_sq:
                    length_matches += 1

            # Check if exactly 2 of each (remaining 2 are diagonals)
            if breadth_matches == 2 and length_matches == 2:
                self.get_logger().info("  > Found a 4-point shape (parallelogram).")
                ordered_pts = self._order_points([p1, p2, p3, p4])
                found_rectangles.append(ordered_pts)

                # Draw the polygon (Yellow)
                pts_array = np.array(ordered_pts, dtype=np.int32).reshape(
                    (-1, 1, 2)
                )
                cv2.polylines(
                    output_img,
                    [pts_array],
                    isClosed=True,
                    color=(0, 255, 255),
                    thickness=2,
                )

                # Add ID number to image
                rect_id = id_counter + len(found_rectangles)
                center_px = np.mean(ordered_pts, axis=0).astype(int)
                cv2.putText(
                    output_img,
                    str(rect_id),
                    (center_px[0] + 5, center_px[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                )

        return found_rectangles, output_img

    def _find_rectangles_from_3_points(
        self,
        points: List[PointTuple],
        output_img: np.ndarray,
        id_counter: int = 0,
    ) -> Tuple[List[RectPoints], np.ndarray]:
        """
        Finds rectangles by checking 3-point right-angle combinations.
        Uses `self.rect_config` and `self.map_config`.

        Args:
            points: List of candidate (x, y) leg points.
            output_img: The OpenCV image to draw visualizations on.
            id_counter: The starting ID number for new rectangles.

        Returns:
            A tuple containing:
            - A list of found rectangles [RectPoints, ...].
            - The annotated output image.
        """
        # Convert real-world meters to squared pixel distances
        res = self.map_config['resolution']
        min_L_sq = (self.rect_config['min_len_m'] / res) ** 2
        max_L_sq = (self.rect_config['max_len_m'] / res) ** 2
        min_B_sq = (self.rect_config['min_breadth_m'] / res) ** 2
        max_B_sq = (self.rect_config['max_breadth_m'] / res) ** 2
        pythag_tol_sq = self.rect_config['pythag_tolerance_sq']

        found_rectangles: List[RectPoints] = []

        if len(points) < 3:
            return [], output_img

        # Check every combination of 3 points
        for combo in itertools.combinations(points, 3):
            p1, p2, p3 = combo

            # Calculate squared distances
            d12_sq = self._dist_sq(p1, p2)
            d13_sq = self._dist_sq(p1, p3)
            d23_sq = self._dist_sq(p2, p3)

            potentials = [
                (p1, p2, p3, d23_sq),  # p1 is corner
                (p2, p1, p3, d13_sq),  # p2 is corner
                (p3, p1, p2, d12_sq),  # p3 is corner
            ]

            for corner, adj1, adj2, hyp_sq in potentials:
                side1_sq = self._dist_sq(corner, adj1)
                side2_sq = self._dist_sq(corner, adj2)

                # 1. Check for right angle (Pythagorean theorem)
                if abs(side1_sq + side2_sq - hyp_sq) > pythag_tol_sq:
                    continue

                # 2. Check if sides match Length and Breadth
                is_L1 = min_L_sq <= side1_sq <= max_L_sq
                is_B1 = min_B_sq <= side1_sq <= max_B_sq
                is_L2 = min_L_sq <= side2_sq <= max_L_sq
                is_B2 = min_B_sq <= side2_sq <= max_B_sq

                if (is_L1 and is_B2) or (is_B1 and is_L2):
                    self.get_logger().info(
                        "  > Found a 3-point shape (rectangle corner)."
                    )

                    # Infer the 4th point
                    p4_x = adj1[0] + adj2[0] - corner[0]
                    p4_y = adj1[1] + adj2[1] - corner[1]
                    p4 = (p4_x, p4_y)

                    all_points = [corner, adj1, adj2, p4]
                    ordered_pts = self._order_points(all_points)
                    found_rectangles.append(ordered_pts)

                    # Draw the full polygon (Magenta)
                    pts_array = np.array(ordered_pts, dtype=np.int32).reshape(
                        (-1, 1, 2)
                    )
                    cv2.polylines(
                        output_img,
                        [pts_array],
                        isClosed=True,
                        color=(255, 0, 255),
                        thickness=2,
                    )

                    # Add ID number to image
                    rect_id = id_counter + len(found_rectangles)
                    center_px = np.mean(ordered_pts, axis=0).astype(int)
                    cv2.putText(
                        output_img,
                        str(rect_id),
                        (center_px[0] + 5, center_px[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 0, 255),
                        2,
                    )
                    break  # Found one, break from checking other corners

        return found_rectangles, output_img

    def _publish_rectangles(self, rect_list: List[RectPoints]):
        """
        Publishes a list of rectangles as a MarkerArray to /detected_tables.
        Uses `self.map_config`.

        Args:
            rect_list: A list of found rectangles [RectPoints, ...].
        """
        marker_array = MarkerArray()

        # Get map parameters from stored config
        res = self.map_config['resolution']
        origin_x = self.map_config['origin_x']
        origin_y = self.map_config['origin_y']
        map_height_px = self.map_config.get('map_height_pixels')

        if map_height_px is None:
            self.get_logger().error(
                "Map height in pixels is not set. "
                "Was _find_table_leg_candidates not run?"
            )
            return
        
        frame_id = "map"  # Assumes this is the correct frame

        self.get_logger().info(
            "\n--- Publishing Table Centers (Map Coordinates) ---"
        )

        for i, rect_pixels in enumerate(rect_list):
            rect_id = i + 1

            # 1. Convert pixel points (top-left) to map coords (bottom-left)
            map_points = []
            for px, py in rect_pixels:
                # PGM (0,0) is top-left. Map (0,0) is bottom-left (rel to origin)
                # mx = (pixel_x * resolution) + map_origin_x
                # my = ((image_height_px - pixel_y) * resolution) + map_origin_y
                mx = (float(px) * res) + origin_x
                my = (float(map_height_px - py) * res) + origin_y
                map_points.append(Point(x=mx, y=my, z=0.0))

            # 2. Calculate center in map coordinates
            center_mx = np.mean([p.x for p in map_points])
            center_my = np.mean([p.y for p in map_points])

            # 3. Create Marker for the rectangle lines (LINE_STRIP)
            line_marker = Marker()
            line_marker.header.frame_id = frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "table_shapes"
            line_marker.id = rect_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            # Add points, closing the loop by adding the first point at the end
            line_marker.points = map_points + [map_points[0]]
            line_marker.scale.x = 0.05  # line width in meters
            line_marker.color.a = 1.0
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0  # Yellow

            # 4. Create Marker for the ID number (TEXT_VIEW_FACING)
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "table_ids"
            text_marker.id = rect_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = center_mx
            text_marker.pose.position.y = center_my
            text_marker.pose.position.z = 0.1  # 10cm above the map
            text_marker.text = str(rect_id)
            text_marker.scale.z = 0.5  # text height in meters
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0  # White

            # 5. Add to array
            marker_array.markers.append(line_marker)
            marker_array.markers.append(text_marker)

            # 6. Print to logger
            self.get_logger().info(
                f"  > Table ID {rect_id} @ (x={center_mx:.2f}, y={center_my:.2f})"
            )

        # Publish the whole array
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            self.get_logger().info(
                f"\nPublished {len(rect_list)} tables to "
                f"/detected_tables topic."
            )
        else:
            self.get_logger().info("\nNo rectangles to publish.")
            # Publish an empty array to clear any old markers
            self.marker_pub.publish(marker_array)

    def _process_map_file(self, map_file_path: str) -> bool:
        """
        The main processing pipeline. Loads, finds, and publishes.

        Args:
            map_file_path: The file path to the PGM map.

        Returns:
            bool: True on success, False on fatal error.
        """
        # --- Get path from parameter ---
        # Check if the provided path is absolute
        if os.path.isabs(map_file_path):
            image_file = map_file_path
            # Use the map's directory for the output file
            base_dir = os.path.dirname(image_file)
        else:
            # If not absolute, assume it's relative to the CWD
            base_dir = os.getcwd()
            image_file = os.path.join(base_dir, map_file_path)

        output_file = os.path.join(
            base_dir, "table_legs_detected_with_rectangles.png"
        )
        self.get_logger().info(f"📂 Using map file: {image_file}")

        try:
            # Stage 1: Find leg candidates
            points, visual_img = self._find_table_leg_candidates(image_file)

            if visual_img is None:
                raise Exception("Image loading failed.")

            self.get_logger().info(
                f"\n--- Found {len(points)} potential table leg candidates ---"
            )
            if points:
                # --- Stage 2a: Find rectangles from 4 candidates ---
                self.get_logger().info(
                    "\n--- STAGE 2a: Checking 4-point combinations "
                    "(Parallelograms) ---"
                )
                rects_4, visual_img_with_rects = (
                    self._find_rectangles_from_points(
                        points, visual_img, id_counter=0
                    )
                )

                # --- De-duplication Logic ---
                used_points_set: Set[PointTuple] = set(
                    itertools.chain.from_iterable(rects_4)
                )
                self.get_logger().info(
                    f"  > Stage 2a used {len(used_points_set)} unique points."
                )

                # Create a new list of points that were *not* used
                remaining_points = [
                    p for p in points if p not in used_points_set
                ]
                self.get_logger().info(
                    f"  > {len(remaining_points)} points remaining for "
                    f"3-point search."
                )

                # --- Stage 2b: Find rectangles from 3 candidates ---
                self.get_logger().info(
                    "\n--- STAGE 2b: Checking 3-point combinations "
                    "(Right-Angle Rectangles) ---"
                )
                id_start_for_3pt = len(rects_4)
                rects_3, visual_img_with_rects = (
                    self._find_rectangles_from_3_points(
                        remaining_points,  # <-- Pass the *filtered* list
                        visual_img_with_rects,
                        id_counter=id_start_for_3pt,
                    )
                )

                total_rectangles = rects_4 + rects_3
                self.get_logger().info(
                    f"\n--- Found {len(total_rectangles)} total shapes ---"
                )

                # --- Stage 3: Publish to ROS ---
                self.get_logger().info("\n--- STAGE 3: Publishing to ROS ---")
                self._publish_rectangles(total_rectangles)
                self.get_logger().info("Giving publisher 0.5s to send...")
                time.sleep(0.5)  # Give the publisher a moment to send

                cv2.imwrite(output_file, visual_img_with_rects)
                self.get_logger().info(
                    f"\n✅ Saved visualization with rectangle lines to "
                    f"{output_file}"
                )

            else:
                self.get_logger().info(
                    "\nNo leg candidates found, so no shapes to fit."
                )
                cv2.imwrite(output_file, visual_img)
                self.get_logger().info(
                    f"\n✅ Saved visualization (no legs found) to {output_file}"
                )
            
            return True

        except FileNotFoundError:
            self.get_logger().error(
                f"\nError: The file '{image_file}' was not found."
            )
            return False
        except Exception as e:
            self.get_logger().error(f"\nAn error occurred: {e}")
            self.get_logger().error(traceback.format_exc())
            return False

    def run_detection(self):
        """
        Public entry point to run the entire detection pipeline.
        
        This method gets the map file path parameter, logs the
        hard-coded configs, and then processes the map file.
        """
        self.get_logger().info("--- Starting Table Detection ---")
        
        # Load the one ROS parameter
        map_path_param = (
            self.get_parameter('map_file_path')
            .get_parameter_value()
            .string_value
        )
        
        # Log the configuration for debugging
        self._log_configuration()

        # Run the main processing pipeline
        if self._process_map_file(map_path_param):
            self.get_logger().info("--- Detection Finished Successfully ---")
        else:
            self.get_logger().error("--- Detection Finished with Errors ---")


# =========================================================================
# --- CONFIGURATIONS ---
# =========================================================================
# All configurations are defined here and passed to the node in main().

# Map resolution from your .yaml file (e.g., 0.05 means 1 pixel = 5cm)
# This is NOT read from the file, you must set it manually.
MAP_CONFIG = {
    'resolution': 0.05,  # meters per pixel
    
    # !! IMPORTANT !!
    # You MUST update these values from your map.yaml file
    # 'origin' is usually [x, y, yaw]
    # Example: origin: [-10.0, -10.0, 0.0]
    'origin_x': 0.0,  # (meters)
    'origin_y': 0.0,  # (meters)
}

# Blob detection parameters (these are still in pixels)
BLOB_CONFIG = {
    'occupancy_threshold': 100,
    'min_area': 0.9,
    'max_area': 10,
    'min_circularity': 0.6
}

# Rectangle search parameters (NOW IN METERS)
RECT_CONFIG = {
    'min_len_m': 1.75,     # e.g., 1.8m table +/- 5cm
    'max_len_m': 1.85,
    'min_breadth_m': 1.25, # e.g., 1.3m table +/- 5cm
    'max_breadth_m': 1.35,
    'pythag_tolerance_sq': 50.0 # Pixel tolerance for right-angle check
}


# =========================================================================
# --- Main execution ---
# =========================================================================
def main(args=None):
    """
    Main entry point for the ROS 2 node.
    """
    rclpy.init(args=args)
    
    # Create the node, passing in the hard-coded config dictionaries
    table_detector_node = TableDetectorNode(
        map_config=MAP_CONFIG,
        blob_config=BLOB_CONFIG,
        rect_config=RECT_CONFIG
    )

    try:
        # Run the detection logic
        # This script runs once, publishes, and then exits
        table_detector_node.run_detection()
    
    except Exception as e:
        table_detector_node.get_logger().error(f"Unhandled exception: {e}")
        table_detector_node.get_logger().error(traceback.format_exc())
    
    finally:
        # Clean up
        table_detector_node.get_logger().info("Detection finished, shutting down.")
        table_detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()