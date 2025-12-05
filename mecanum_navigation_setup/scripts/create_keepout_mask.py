#!/usr/bin/env python3
"""
Create a keepout mask from your map image.
Mark tape boundaries and forbidden areas as occupied (black).
"""

import cv2
import numpy as np
import yaml
import argparse
from pathlib import Path


def create_keepout_mask(map_image_path, output_dir, tape_boundaries=None):
    """
    Create keepout mask from map image
    
    Args:
        map_image_path: Path to original map PGM file
        output_dir: Directory to save keepout mask
        tape_boundaries: List of (x1, y1, x2, y2, thickness) tuples for tape lines
    """
    # Read the original map
    map_img = cv2.imread(str(map_image_path), cv2.IMREAD_GRAYSCALE)
    
    if map_img is None:
        raise ValueError(f"Could not read map image: {map_image_path}")
    
    print(f"Original map size: {map_img.shape}")
    
    # Create empty mask (all white = free space)
    mask = np.ones_like(map_img) * 254  # 254 = free space
    
    # Add tape boundaries as keepout zones (black = occupied)
    if tape_boundaries:
        for boundary in tape_boundaries:
            x1, y1, x2, y2, thickness = boundary
            cv2.line(mask, (x1, y1), (x2, y2), 0, thickness)
            print(f"Added boundary: ({x1},{y1}) -> ({x2},{y2}), thickness={thickness}")
    
    # Save the keepout mask
    output_path = Path(output_dir) / "keepout_mask.pgm"
    cv2.imwrite(str(output_path), mask)
    print(f"Saved keepout mask to: {output_path}")
    
    return output_path


def create_keepout_yaml(keepout_pgm_path, original_yaml_path, output_dir):
    """
    Create YAML file for keepout mask (matching original map metadata)
    """
    # Read original map YAML
    with open(original_yaml_path, 'r') as f:
        original_config = yaml.safe_load(f)
    
    # Create keepout YAML with same metadata
    keepout_config = {
        'image': 'keepout_mask.pgm',
        'resolution': original_config['resolution'],
        'origin': original_config['origin'],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    # Save keepout YAML
    output_yaml = Path(output_dir) / "keepout_mask.yaml"
    with open(output_yaml, 'w') as f:
        yaml.dump(keepout_config, f, default_flow_style=False)
    
    print(f"Saved keepout YAML to: {output_yaml}")
    return output_yaml


def interactive_boundary_selection(map_image_path, window_width=None, window_height=None):
    """
    Interactive tool to select tape boundaries by clicking on the map
    """
    map_img = cv2.imread(str(map_image_path), cv2.IMREAD_GRAYSCALE)
    
    if map_img is None:
        raise ValueError(f"Could not read map image: {map_image_path}")
    
    # Get original map dimensions
    orig_height, orig_width = map_img.shape
    print(f"Map dimensions: {orig_width}x{orig_height}")
    
    # Determine display size
    if window_width is None or window_height is None:
        # Auto-scale to fit screen while maintaining aspect ratio
        screen_width = 1920   # Adjust for your monitor
        screen_height = 1080  # Adjust for your monitor
        
        scale = min(screen_width / orig_width, screen_height / orig_height)
        scale = min(scale, 1.0)  # Don't upscale if map is already small
        scale = max(scale, 0.3)  # Minimum 30% of original size
        
        window_width = int(orig_width * scale)
        window_height = int(orig_height * scale)
    
    print(f"Display window size: {window_width}x{window_height}")
    print(f"Scale factor: {window_width/orig_width:.2f}x")
    
    # Resize for display
    display_img = cv2.resize(map_img, (window_width, window_height), interpolation=cv2.INTER_LINEAR)
    map_colored = cv2.cvtColor(display_img, cv2.COLOR_GRAY2BGR)
    
    # Store original colored image for reset
    original_colored = map_colored.copy()
    
    # Scale factor for coordinate conversion
    scale_x = orig_width / window_width
    scale_y = orig_height / window_height
    
    boundaries = []
    current_line = []
    zoom_level = 1.0
    pan_x = 0
    pan_y = 0
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal current_line, boundaries, map_colored
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Convert display coordinates to original map coordinates
            orig_x = int(x * scale_x)
            orig_y = int(y * scale_y)
            
            current_line.append((orig_x, orig_y))
            
            # Draw on display image
            cv2.circle(map_colored, (x, y), 5, (0, 255, 0), -1)
            
            if len(current_line) == 2:
                # Draw line on display
                display_x1 = int(current_line[0][0] / scale_x)
                display_y1 = int(current_line[0][1] / scale_y)
                display_x2 = int(current_line[1][0] / scale_x)
                display_y2 = int(current_line[1][1] / scale_y)
                cv2.line(map_colored, (display_x1, display_y1), (display_x2, display_y2), (0, 0, 255), 3)
                
                # Ask for thickness
                print(f"\nLine from ({current_line[0][0]},{current_line[0][1]}) to ({current_line[1][0]},{current_line[1][1]})")
                print(f"Length: {np.sqrt((current_line[1][0]-current_line[0][0])**2 + (current_line[1][1]-current_line[0][1])**2):.0f} pixels")
                thickness = int(input("Enter line thickness in pixels (e.g., 10): "))
                
                # Store boundary (in original coordinates)
                x1, y1 = current_line[0]
                x2, y2 = current_line[1]
                boundaries.append((x1, y1, x2, y2, thickness))
                
                print(f"✓ Added boundary #{len(boundaries)}: ({x1},{y1}) -> ({x2},{y2}), thickness={thickness}")
                
                # Draw thicker line to show final result
                cv2.line(map_colored, (display_x1, display_y1), (display_x2, display_y2), 
                        (255, 0, 0), max(3, int(thickness / scale_x)))
                
                # Reset for next line
                current_line = []
            
            cv2.imshow('Select Tape Boundaries', map_colored)
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click to undo last boundary
            if boundaries:
                removed = boundaries.pop()
                print(f"✗ Removed boundary: {removed}")
                # Redraw everything
                map_colored = original_colored.copy()
                for bnd in boundaries:
                    x1, y1, x2, y2, thick = bnd
                    display_x1 = int(x1 / scale_x)
                    display_y1 = int(y1 / scale_y)
                    display_x2 = int(x2 / scale_x)
                    display_y2 = int(y2 / scale_y)
                    cv2.line(map_colored, (display_x1, display_y1), (display_x2, display_y2), 
                            (255, 0, 0), max(3, int(thick / scale_x)))
                cv2.imshow('Select Tape Boundaries', map_colored)
        
        elif event == cv2.EVENT_MOUSEWHEEL:
            # Mouse wheel zoom (future enhancement)
            pass
    
    # Create window with specific size
    cv2.namedWindow('Select Tape Boundaries', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Select Tape Boundaries', window_width, window_height)
    cv2.setMouseCallback('Select Tape Boundaries', mouse_callback)
    
    print("\n" + "="*60)
    print("INTERACTIVE BOUNDARY SELECTION")
    print("="*60)
    print("\nInstructions:")
    print("  • LEFT CLICK:  Place point (need 2 points to make a line)")
    print("  • RIGHT CLICK: Undo last boundary")
    print("  • 'q' key:     Quit and save boundaries")
    print("  • 'r' key:     Reset all boundaries")
    print("  • '+' key:     Zoom in (experimental)")
    print("  • '-' key:     Zoom out (experimental)")
    print("  • 's' key:     Save current view as preview")
    print("="*60)
    print()
    
    # Show initial view
    cv2.imshow('Select Tape Boundaries', map_colored)
    
    while True:
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            if boundaries:
                print(f"\n✓ Saving {len(boundaries)} boundaries...")
            else:
                print("\n⚠ No boundaries defined!")
            break
        
        elif key == ord('r'):
            boundaries = []
            current_line = []
            map_colored = original_colored.copy()
            cv2.imshow('Select Tape Boundaries', map_colored)
            print("↻ Reset all boundaries")
        
        elif key == ord('s'):
            preview_path = Path("keepout_preview.png")
            cv2.imwrite(str(preview_path), map_colored)
            print(f"💾 Saved preview to: {preview_path}")
        
        elif key == ord('+') or key == ord('='):
            zoom_level *= 1.2
            print(f"🔍 Zoom: {zoom_level:.1f}x")
        
        elif key == ord('-') or key == ord('_'):
            zoom_level /= 1.2
            print(f"🔍 Zoom: {zoom_level:.1f}x")
        
        elif key == 27:  # ESC key
            print("\n✗ Cancelled")
            boundaries = []
            break
    
    cv2.destroyAllWindows()
    
    # Print summary
    if boundaries:
        print("\n" + "="*60)
        print("BOUNDARIES SUMMARY")
        print("="*60)
        for i, (x1, y1, x2, y2, thick) in enumerate(boundaries, 1):
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            print(f"{i}. ({x1},{y1}) → ({x2},{y2}) | thickness={thick}px | length={length:.0f}px")
        print("="*60 + "\n")
    
    return boundaries


def main():
    parser = argparse.ArgumentParser(description='Create keepout mask for Nav2')
    parser.add_argument('--map', type=str, required=True,
                       help='Path to original map PGM file')
    parser.add_argument('--map-yaml', type=str, required=True,
                       help='Path to original map YAML file')
    parser.add_argument('--output-dir', type=str, default='.',
                       help='Output directory for keepout mask')
    parser.add_argument('--interactive', action='store_true',
                       help='Interactive boundary selection')
    parser.add_argument('--window-width', type=int, default=None,
                       help='Display window width (default: auto-scale)')
    parser.add_argument('--window-height', type=int, default=None,
                       help='Display window height (default: auto-scale)')
    parser.add_argument('--boundaries', type=str,
                       help='Boundaries as JSON: [[x1,y1,x2,y2,thick],...]')
    
    args = parser.parse_args()
    
    # Get tape boundaries
    if args.interactive:
        print("Starting interactive boundary selection...")
        boundaries = interactive_boundary_selection(
            args.map, 
            args.window_width, 
            args.window_height
        )
        
        if not boundaries:
            print("No boundaries defined. Exiting.")
            return
    elif args.boundaries:
        import json
        boundaries = json.loads(args.boundaries)
    else:
        # Example: horizontal line at y=250, vertical line at x=100
        # Adjust these based on your actual map
        boundaries = [
            # (x1, y1, x2, y2, thickness)
            (0, 250, 500, 250, 10),    # Horizontal tape at y=250
            (100, 0, 100, 500, 10),    # Vertical tape at x=100
        ]
        print("Using default boundaries. Use --interactive for custom selection.")
    
    # Create keepout mask
    keepout_pgm = create_keepout_mask(args.map, args.output_dir, boundaries)
    
    # Create keepout YAML
    keepout_yaml = create_keepout_yaml(keepout_pgm, args.map_yaml, args.output_dir)
    
    print("\n" + "="*60)
    print("KEEPOUT MASK CREATED SUCCESSFULLY!")
    print("="*60)
    print(f"📄 Mask image: {keepout_pgm}")
    print(f"📄 Mask YAML:  {keepout_yaml}")
    print(f"📊 Boundaries: {len(boundaries)} line(s)")
    print("\nNext steps:")
    print("  1. Review keepout_preview.png (if saved)")
    print("  2. Copy files to mecanum_navigation_setup/maps/")
    print("  3. Update nav2_params.yaml")
    print("  4. Launch navigation with keepout zones")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()