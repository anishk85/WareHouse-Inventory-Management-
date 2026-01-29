import cv2
import torch
import numpy as np
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

# --- CONFIGURATION ---
# CRITICAL FIX: Added '-hf' suffix so it works with the 'transformers' library
MODEL_ID = "depth-anything/Depth-Anything-V2-Small-hf"
VIDEO_SOURCE = 1

# Safety Threshold (0 to 255). 
# Higher = Detects objects closer to camera.
COLLISION_THRESHOLD = 230 
WARNING_THRESHOLD = 180

# --- SETUP ---
print(f"Loading {MODEL_ID}...")
try:
    processor = AutoImageProcessor.from_pretrained(MODEL_ID)
    model = AutoModelForDepthEstimation.from_pretrained(MODEL_ID)
except OSError as e:
    print(f"\nERROR: Could not load model. Make sure you are using the '-hf' version.\nDetails: {e}")
    exit(1)

# Move to GPU if available
device = "cuda" if torch.cuda.is_available() else "cpu"
model.to(device)
print(f"Model loaded on {device}")

cap = cv2.VideoCapture(VIDEO_SOURCE)

if not cap.isOpened():
    print(f"Error: Could not open video source {VIDEO_SOURCE}")
    exit(1)

print("Starting Depth Stream. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret: 
        break
    
    # 1. Resize for speed 
    # (Depth estimation is heavy; 480x360 is a good balance for V2-Small)
    # We resize preserving aspect ratio or just fixed size if speed is priority
    process_height, process_width = 360, 480
    small_frame = cv2.resize(frame, (process_width, process_height))
    
    # 2. Prepare Inputs
    inputs = processor(images=small_frame, return_tensors="pt").to(device)
    
    # 3. Inference
    with torch.no_grad():
        outputs = model(**inputs)
        predicted_depth = outputs.predicted_depth

    # 4. Interpolate to original processing size
    # We interpolate back to 'small_frame' size to keep visualization fast
    prediction = torch.nn.functional.interpolate(
        predicted_depth.unsqueeze(1),
        size=small_frame.shape[:2],
        mode="bicubic",
        align_corners=False,
    )
    
    # 5. Normalize Depth (0-255)
    depth_output = prediction.squeeze().cpu().numpy()
    
    # Min-Max Normalization to map depth to 0-255 range
    # In this model: Higher pixel value = CLOSER
    depth_min = depth_output.min()
    depth_max = depth_output.max()
    
    # Avoid division by zero
    if depth_max - depth_min > 0:
        depth_uint8 = (255 * (depth_output - depth_min) / (depth_max - depth_min)).astype("uint8")
    else:
        depth_uint8 = np.zeros_like(depth_output, dtype="uint8")

    # 6. Collision Logic (Center ROI)
    h, w = depth_uint8.shape
    roi_size = 300 # Size of the "detection box" in the center
    center_y, center_x = h // 2, w // 2
    
    # Extract Region of Interest
    y1 = max(0, center_y - roi_size//2)
    y2 = min(h, center_y + roi_size//2)
    x1 = max(0, center_x - roi_size//2)
    x2 = min(w, center_x + roi_size//2)
    
    roi = depth_uint8[y1:y2, x1:x2]
    
    # Use 95th percentile to be robust against noise, but sensitive to obstacles
    if roi.size > 0:
        avg_closeness = np.percentile(roi, 95)
    else:
        avg_closeness = 0

    # 7. Visualization
    color_depth = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_MAGMA)
    
    status = "PATH CLEAR"
    status_color = (0, 255, 0) # Green
    
    if avg_closeness > COLLISION_THRESHOLD:
        status = "CRITICAL STOP!"
        status_color = (0, 0, 255) # Red
    elif avg_closeness > WARNING_THRESHOLD:
        status = "WARNING"
        status_color = (0, 255, 255) # Yellow

    # Draw ROI Box
    cv2.rectangle(color_depth, (x1, y1), (x2, y2), (255, 255, 255), 2)
    
    # Overlay Info
    cv2.putText(color_depth, f"{status}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
    cv2.putText(color_depth, f"Score: {int(avg_closeness)}", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    # Show Windows
    cv2.imshow("Depth Perception", color_depth)
    # cv2.imshow("RGB Feed", small_frame) # Optional: Comment out to save resources
    
    if cv2.waitKey(1) == ord('q'): 
        break

cap.release()
cv2.destroyAllWindows()
