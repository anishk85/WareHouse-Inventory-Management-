import cv2
import numpy as np

# --- CONFIGURATION ---
# 0 for webcam, or provide video path e.g. 'lift_test.mp4'
# Since you have multiple streams, try 0, 2, or 4 based on your screenshot
VIDEO_SOURCE = 1

# Resize for speed (Optical Flow is heavy on CPU)
FRAME_WIDTH = 320 
MAGNITUDE_THRESHOLD = 12.0  # Minimum speed to count as "motion" (filters noise)
MIN_AREA = 800              # Minimum cluster size to trigger alarm

# --- SETUP ---
cap = cv2.VideoCapture(VIDEO_SOURCE)
ret, frame1 = cap.read()
if not ret:
    print("Failed to grab first frame")
    exit()

frame1 = cv2.resize(frame1, (FRAME_WIDTH, int(frame1.shape[0] * (FRAME_WIDTH / frame1.shape[1]))))
prvs = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[..., 1] = 255 # Saturation to max

print("Press 'q' to quit.")
print("Press 'c' to toggle viewing the Color Flow (Debug Mode).")

show_color_flow = False

while True:
    ret, frame2 = cap.read()
    if not ret:
        break

    # Resize to reduce computation load
    frame2 = cv2.resize(frame2, (FRAME_WIDTH, int(frame2.shape[0] * (FRAME_WIDTH / frame2.shape[1]))))
    next_frame = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # 1. Calculate Dense Optical Flow (Farneback Algorithm)
    flow = cv2.calcOpticalFlowFarneback(prvs, next_frame, None, 
                                        0.5, 3, 15, 3, 5, 1.2, 0)

    # 2. Convert Flow to Polar coordinates (Magnitude and Angle)
    # mag: Speed of pixel | ang: Direction of pixel (0 to 360 degrees)
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1], angleInDegrees=True)

    # 3. Create the "Alarm Mask"
    # Step A: Filter out noise (very slow movement)
    motion_mask = mag > MAGNITUDE_THRESHOLD

    # Step B: Filter out the Lift's Motion (The "Safe Angles")
    # You must TUNE these ranges. 
    # Example: If lift moves UP (90 deg), ignore 80-100.
    # Note: Scissor lifts also have horizontal motion on the arms. 
    # You might need to block two ranges (e.g., Up AND slightly Inward).
    
    # Let's assume for now we ignore purely vertical motion (Up/Down)
    # Up is approx 90, Down is approx 270.
    angle_tolerance = 25
    
    ignore_up = (ang > (90 - angle_tolerance)) & (ang < (90 + angle_tolerance))
    ignore_down = (ang > (270 - angle_tolerance)) & (ang < (270 + angle_tolerance))
    
    # Combine ignores
    ignore_mask = ignore_up | ignore_down
    
    # Apply filter: Keep motion that is FAST enough AND NOT in the ignore directions
    alarm_mask = motion_mask & (~ignore_mask)
    
    # Clean up the mask with morphology (remove single pixel speckles)
    alarm_mask_uint8 = alarm_mask.astype(np.uint8) * 255
    kernel = np.ones((3,3), np.uint8)
    alarm_mask_uint8 = cv2.morphologyEx(alarm_mask_uint8, cv2.MORPH_OPEN, kernel)

    # 4. Find Contours in the Alarm Mask (Where is the obstacle?)
    contours, _ = cv2.findContours(alarm_mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    intrusion_detected = False
    for contour in contours:
        if cv2.contourArea(contour) > MIN_AREA:
            intrusion_detected = True
            x, y, w, h = cv2.boundingRect(contour)
            # Draw red box around the intrusion
            cv2.rectangle(frame2, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(frame2, "INTRUSION", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Status Display
    status_text = "Status: SAFE"
    color = (0, 255, 0)
    if intrusion_detected:
        status_text = "Status: WARNING - OBJ DETECTED"
        color = (0, 0, 255)
    
    cv2.putText(frame2, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Visualization Logic
    if show_color_flow:
        # Map angle to Hue for colorful debug view
        hsv[..., 0] = ang / 2
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        bgr_flow = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.imshow('Debug: Color Flow', bgr_flow)
        cv2.imshow('Debug: Alarm Mask', alarm_mask_uint8)
    
    cv2.imshow('Lift Feed', frame2)

    prvs = next_frame

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        show_color_flow = not show_color_flow
        if not show_color_flow:
            cv2.destroyWindow('Debug: Color Flow')
            cv2.destroyWindow('Debug: Alarm Mask')

cap.release()
cv2.destroyAllWindows()