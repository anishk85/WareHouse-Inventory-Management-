import qrcode
from PIL import Image

# Data to encode
data = "https://example.com"

# Create QR code instance
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=10,
    border=4,
)

# Add data
qr.add_data(data)
qr.make(fit=True)

# Create image
img = qr.make_image(fill_color="black", back_color="white")

# Resize to 5 cm × 5 cm
# 1 inch = 2.54 cm → 5 cm = 5 / 2.54 = 1.9685 inches
# At 100 DPI → pixels = inches * DPI ≈ 197 px
target_size = (200, 200)  # width, height
img = img.resize(target_size, Image.NEAREST)

# Save file
img.save("qrcode_5cm.png")

print("QR code saved as qrcode_5cm.png")
