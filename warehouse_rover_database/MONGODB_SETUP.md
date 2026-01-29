# Warehouse Rover Database - MongoDB Atlas Setup

## 📚 Overview

This package provides a **MongoDB Atlas** cloud-based database system for the Warehouse Rover, replacing the previous SQLite implementation. It stores QR code detections from warehouse missions in the cloud for easy access and analysis.

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    MongoDB Atlas Cloud                  │
│  ┌──────────────────┐      ┌──────────────────────┐   │
│  │    missions      │      │     detections        │   │
│  │  - mission_id    │      │  - mission_id         │   │
│  │  - start_time    │      │  - rack_id            │   │
│  │  - end_time      │      │  - shelf_id           │   │
│  │  - status        │      │  - item_code          │   │
│  │  - total_detect  │      │  - confidence         │   │
│  └──────────────────┘      │  - timestamp          │   │
│                            │  - qr_data            │   │
│                            └──────────────────────┘   │
└─────────────────────────────────────────────────────────┘
                              ▲
                              │ pymongo
                              │
              ┌───────────────┴───────────────┐
              │   inventory_node_mongo.py     │
              │   (ROS2 Python Node)          │
              └───────────────┬───────────────┘
                              ▲
                              │ /qr_detections
                              │
              ┌───────────────┴───────────────┐
              │   QR Detection System         │
              │   (Camera + Processing)       │
              └───────────────────────────────┘
```

---

## 🚀 Quick Start

### 1. Create MongoDB Atlas Account (FREE)

1. Go to [MongoDB Atlas](https://www.mongodb.com/cloud/atlas)
2. Sign up for a **FREE** account
3. Create a new cluster (FREE tier M0 is sufficient)
4. Wait for cluster to be created (~5 minutes)

### 2. Get Connection String

1. Click **"Connect"** on your cluster
2. Choose **"Connect your application"**
3. Select **Python** and version **3.6 or later**
4. Copy the connection string:
   ```
   mongodb+srv://<username>:<password>@<cluster>.mongodb.net/
   ```
5. Replace `<password>` with your actual password

### 3. Setup Database Access

1. In Atlas, go to **"Database Access"**
2. Add a database user with username and password
3. In **"Network Access"**, add your IP address (or `0.0.0.0/0` for anywhere - less secure)

### 4. Install Python Dependencies

```bash
# Install pymongo and DNS resolver
pip3 install pymongo dnspython

# Or use requirements file
cd /root/ros2_ws/src/warehouse_rover_database
pip3 install -r requirements.txt
```

### 5. Set Connection String

**Option A: Environment Variable (Recommended)**
```bash
# Add to ~/.bashrc or set before running
export MONGODB_CONNECTION_STRING="mongodb+srv://<username>:<password>@<cluster>.mongodb.net/"
```

**Option B: Edit Config File**
```bash
# Edit config/mongodb_params.yaml
nano /root/ros2_ws/src/warehouse_rover_database/config/mongodb_params.yaml

# Set mongodb_connection_string parameter
```

**Option C: Pass as Launch Argument**
```bash
ros2 launch warehouse_rover_database mongodb_inventory.launch.py \
    mongodb_connection_string:="mongodb+srv://..."
```

### 6. Build and Run

```bash
# Build the package
cd /root/ros2_ws
colcon build --packages-select warehouse_rover_database
source install/setup.bash

# Run the inventory node
ros2 launch warehouse_rover_database mongodb_inventory.launch.py
```

---

## 🧪 Testing

### Test MongoDB Connection (No ROS2)

```bash
cd /root/ros2_ws/src/warehouse_rover_database/warehouse_rover_database

# Set connection string
export MONGODB_CONNECTION_STRING="mongodb+srv://..."

# Run test
python3 mongodb_manager.py
```

Expected output:
```
============================================================
  MongoDB Manager Test
============================================================

✓ Connected to MongoDB: warehouse_rover

1. Starting test mission...
✓ Mission started: TEST_MISSION_001

2. Adding test detections...

3. Getting statistics...
   Total detections: 3
   Average confidence: 0.917
   Unique racks: 2

4. Ending mission...
✓ Mission ended: TEST_MISSION_001 (3 detections)

5. Exporting to JSON...
✓ Exported to: /tmp/test_export.json

6. Getting mission summary...
   Status: completed
   Total detections: 3

✓ MongoDB connection closed
============================================================
✓ All tests passed!
============================================================
```

### Test with Local MongoDB (Optional)

If you don't want to use Atlas, install MongoDB locally:

```bash
# Install MongoDB (Ubuntu)
sudo apt-get update
sudo apt-get install -y mongodb

# Start MongoDB
sudo systemctl start mongodb

# Run with local MongoDB
ros2 launch warehouse_rover_database mongodb_inventory.launch.py use_local:=true
```

---

## 📝 Usage Examples

### Launch with Custom Settings

```bash
# Use Atlas with custom export directory
ros2 launch warehouse_rover_database mongodb_inventory.launch.py \
    export_dir:=/home/rover/mission_exports

# Use local MongoDB
ros2 launch warehouse_rover_database mongodb_inventory.launch.py \
    use_local:=true
```

### Monitor Topics

```bash
# View QR detections being received
ros2 topic echo /qr_detections

# Check node status
ros2 node info /inventory_node
```

### Access Data from MongoDB Atlas

1. Go to your Atlas cluster
2. Click **"Browse Collections"**
3. View data in:
   - `warehouse_rover.missions` - Mission records
   - `warehouse_rover.detections` - QR detection data

### Export Mission Data

```bash
# Exports are automatically created on node shutdown in:
/tmp/inventory_exports/MISSION_<timestamp>.json

# View exported JSON
cat /tmp/inventory_exports/MISSION_*.json | jq
```

---

## 🔧 Configuration

### Parameters (`config/mongodb_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mongodb_connection_string` | `""` | MongoDB Atlas URI |
| `database_name` | `warehouse_rover` | Database name |
| `auto_export` | `true` | Export on shutdown |
| `export_dir` | `/tmp/inventory_exports` | Export directory |
| `use_local_mongodb` | `false` | Use local MongoDB |

---

## 📊 Data Schema

### Missions Collection

```json
{
  "mission_id": "MISSION_20251207_143052",
  "start_time": "2025-12-07T14:30:52.123Z",
  "end_time": "2025-12-07T15:45:30.456Z",
  "status": "completed",
  "total_detections": 42,
  "metadata": {
    "created_by": "warehouse_rover",
    "version": "1.0"
  }
}
```

### Detections Collection

```json
{
  "mission_id": "MISSION_20251207_143052",
  "rack_id": "R01",
  "shelf_id": "S2",
  "item_code": "ITEM12345",
  "qr_data": "R01-S2-ITEM12345",
  "confidence": 0.95,
  "timestamp": "2025-12-07T14:35:12.789Z",
  "metadata": {
    "center_x": 320.5,
    "center_y": 240.8,
    "size_pixels": 150,
    "processing_time_ms": 45.2,
    "camera_height": 1.2
  }
}
```

---

## 🔒 Security Best Practices

1. **Never commit connection strings** to Git
2. Use environment variables for credentials
3. Create **read-only users** for analytics
4. Set **IP whitelist** in Atlas Network Access
5. Enable **encryption at rest** (free in Atlas)
6. Use **strong passwords** for database users

---

## 🐛 Troubleshooting

### Connection Errors

```bash
# Test DNS resolution
ping <your-cluster>.mongodb.net

# Check firewall
sudo ufw status
```

### Import Errors

```bash
# Verify pymongo installation
python3 -c "import pymongo; print(pymongo.__version__)"

# Reinstall if needed
pip3 install --upgrade pymongo dnspython
```

### Permission Errors

```bash
# Check export directory permissions
ls -la /tmp/inventory_exports
sudo chmod 777 /tmp/inventory_exports
```

---

## 📈 Querying Data

### Using MongoDB Compass (GUI)

1. Download [MongoDB Compass](https://www.mongodb.com/products/compass)
2. Connect using your connection string
3. Browse collections visually

### Using Python

```python
from warehouse_rover_database.mongodb_manager import MongoDBManager

db = MongoDBManager()

# Get mission summary
summary = db.get_mission_summary("MISSION_20251207_143052")

# Get all detections for a rack
detections = db.get_detections_by_rack("R01")

# Get statistics
stats = db.get_statistics()
print(f"Total detections: {stats['total_detections']}")
```

---

## 🔄 Migration from SQLite

If you have existing SQLite data:

```bash
# Export from SQLite
sqlite3 /tmp/warehouse_inventory.db ".dump" > backup.sql

# Parse and import to MongoDB (script not provided, manual process)
# Or start fresh with new missions
```

---

## 🎯 Next Steps

1. ✅ Setup MongoDB Atlas
2. ✅ Configure connection string
3. ✅ Test connection
4. ✅ Run inventory node
5. ✅ Integrate with QR detection
6. ✅ View data in Atlas dashboard
7. ✅ Create analytics queries

---

## 📞 Support

For issues:
- Check Atlas dashboard for cluster status
- Review ROS2 logs: `ros2 node info /inventory_node`
- Test connection: `python3 mongodb_manager.py`

---

**Made with ❤️ for Warehouse Automation**
