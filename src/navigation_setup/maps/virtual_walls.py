#!/usr/bin/env python3

# This is not a Ros Node or anything this for detecting walls and drawing virtual walls at a particular distance

import cv2
import numpy as np
import json
import os
import math

ROOT = os.path.dirname(os.path.abspath(__file__))
MAP_PGM = os.path.join(ROOT, "map1.pgm")
FINAL_MAP = os.path.join(ROOT, "map.pgm")      
FINAL_JSON = 'final_wall_segments.json'           

wall_offsets = [5, 15, 15]   
reverse_normal = [0, 0, 0]    
thickness = 1            


def to_pt(pt):
    return (int(pt[0]), int(pt[1]))


def main():

    if not os.path.exists(MAP_PGM):
        print(f"Error: map not found at {MAP_PGM}")
        return

    img_color = cv2.imread(MAP_PGM)
    if img_color is None:
        print("Error: failed to read image.")
        return

    img_h, img_w = img_color.shape[:2]
    gray = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)

    _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("No contours found.")
        return

    main_cnt = max(contours, key=cv2.contourArea)

    output = img_color.copy()
    cv2.drawContours(output, [main_cnt], -1, (255, 0, 0), 2)

    rect = cv2.minAreaRect(main_cnt)
    box = cv2.boxPoints(rect)
    corners = np.intp(box)

    wall_segments = []
    wall_waypoints = []

    for i in range(4):
        p1 = tuple(corners[i])
        p2 = tuple(corners[(i + 1) % 4])

        mid = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)

        wall_segments.append({
            "p1": to_pt(p1),
            "p2": to_pt(p2),
            "mid": [mid[0], mid[1]]
        })

        wall_waypoints.append(mid)

        cv2.line(output, p1, p2, (0, 255, 255), 4)
        cv2.putText(output, f"Wall{i+1}", (int(mid[0]), int(mid[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


    virtual_img = gray.copy()

    new_walls = []

    for i in range(len(wall_offsets)):
        w = wall_segments[i]

        x1, y1 = w["p1"]
        x2, y2 = w["p2"]

        offset_px = wall_offsets[i]

        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx * dx + dy * dy)
        if length == 0:
            continue

        nx = -dy / length
        ny = dx / length

        if reverse_normal[i] == 1:
            nx = -nx
            ny = -ny

        ox = nx * offset_px
        oy = ny * offset_px

        p1_shift = (int(x1 + ox), int(y1 + oy))
        p2_shift = (int(x2 + ox), int(y2 + oy))

        cv2.line(virtual_img, p1_shift, p2_shift, color=0, thickness=thickness)

        mx = (p1_shift[0] + p2_shift[0]) / 2
        my = (p1_shift[1] + p2_shift[1]) / 2

        new_walls.append({
            "p1": [p1_shift[0], p1_shift[1]],
            "p2": [p2_shift[0], p2_shift[1]],
            "mid": [mx, my]
        })

    last_wall = wall_segments[-1]

    new_walls.append({
        "p1": last_wall["p1"],
        "p2": last_wall["p2"],
        "mid": last_wall["mid"]
    })

    final_json = {
        "img_w": img_w,
        "img_h": img_h,
        "corners": [to_pt(c) for c in corners],
        "walls": new_walls
    }

    with open(FINAL_JSON, "w") as f:
        json.dump(final_json, f, indent=2)

    print("Saved final JSON:", FINAL_JSON)

    cv2.imwrite(FINAL_MAP, virtual_img)
    print("Saved final modified map:", FINAL_MAP)

    print("\n=== DONE: Full pipeline completed successfully ===")

if __name__ == "__main__":
    main()
