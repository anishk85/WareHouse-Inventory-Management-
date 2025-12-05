# Array of QR detections from a single camera frame

std_msgs/Header header

# Optional context (can be set by other nodes)
string expected_rack_id     # Expected rack ID (for validation)
float32 camera_height       # Camera height in meters

# Detection results
int32 detection_count
QRDetection[] detections

# Performance metrics
float32 processing_time_ms
