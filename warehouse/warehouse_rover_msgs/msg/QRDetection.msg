# Single QR code detection with parsed data

# Raw QR data
string qr_data              # Raw string from QR code

# Parsed fields (populated if valid format)
string rack_id              # e.g., "R01", "R02"
string shelf_id             # e.g., "S1", "S2", "S3"
string item_code            # e.g., "ITEM001", "ITM430"

# Detection quality metrics
float32 confidence          # 0.0 to 1.0
geometry_msgs/Point center  # Center position in image (pixels)
int32 size_pixels          # Approximate QR size in pixels

# Validation status
bool is_valid              # True if QR format is correct
string error_message       # Empty if valid, error description otherwise
