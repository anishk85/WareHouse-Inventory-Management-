import os

# 1. content for model.config
config_content = """<?xml version="1.0"?>
<model>
  <name>qr_codes</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>User</name>
    <email>user@email.com</email>
  </author>
  <description>A collection of QR textures</description>
</model>
"""

# 2. content for a dummy model.sdf (required to be a valid model package)
sdf_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="qr_codes">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.1 0.1 0.001</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

# Write files to the qr_codes directory
base_dir = "qr_codes"
if not os.path.exists(base_dir):
    os.makedirs(base_dir)

with open(f"{base_dir}/model.config", "w") as f:
    f.write(config_content)

with open(f"{base_dir}/model.sdf", "w") as f:
    f.write(sdf_content)

print(f"✅ Success! 'qr_codes' is now a valid Gazebo model package.")
print("You can now use <albedo_map>model://qr_codes/qrcode1.png</albedo_map>")