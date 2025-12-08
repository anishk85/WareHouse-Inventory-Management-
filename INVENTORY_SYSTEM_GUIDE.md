# 🤖 Autonomous Warehouse Inventory System

## Complete System Overview

This system enables fully autonomous inventory management using waypoint navigation, QR code detection, and automatic database updates.

---

## 🎯 Two-Phase Operation

### **Phase 1: Waypoint Creation** 📍
Drive robot manually and save waypoint positions

### **Phase 2: Autonomous Mission** 🚀
Robot automatically visits waypoints, scans QR codes, updates database

---

## 📋 Phase 1: Creating Waypoints

### Step 1: Start Waypoint Creation Mode

```bash
# Terminal 1: Launch waypoint creation system
ros2 launch mecanum_hardware waypoint_creation.launch.py \
    map:=/path/to/your/warehouse_map.yaml \
    waypoints_output:=/tmp/my_waypoints.yaml
```

### Step 2: Set Initial Pose in RViz
1. Open RViz: `ros2 launch nav2_bringup rviz_launch.py`
2. Click "2D Pose Estimate" button
3. Click on map where robot is located
4. Drag to set robot orientation

### Step 3: Drive and Save Waypoints
- **Left Stick Y**: Move forward/backward
- **Right Stick X**: Rotate
- **A Button (Button 0)**: **SAVE WAYPOINT** at current position

```
Drive to Rack 1 → Press A → "Waypoint 1 saved!"
Drive to Rack 2 → Press A → "Waypoint 2 saved!"
Drive to Rack 3 → Press A → "Waypoint 3 saved!"
...
```

### Step 4: Verify Waypoints

```bash
# Check saved waypoints
cat /tmp/my_waypoints.yaml
```

Expected format:
```yaml
waypoints:
  - name: waypoint_1
    position: {x: 2.5, y: 1.2, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
    timestamp: '2025-12-07T15:30:45'
  - name: waypoint_2
    position: {x: 5.1, y: 3.8, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -0.383, w: 0.924}
    timestamp: '2025-12-07T15:32:10'
```

---

## 🚀 Phase 2: Autonomous Inventory Mission

### Step 1: Start Inventory Mission

```bash
# Terminal 1: Launch complete autonomous system
ros2 launch mecanum_hardware inventory_mission.launch.py \
    map:=/path/to/your/warehouse_map.yaml \
    waypoints:=/tmp/my_waypoints.yaml \
    scan_duration:=50.0
```

### What Happens Automatically:

```
┌─────────────────────────────────────────┐
│  MISSION START                          │
│  1. Enable QR Detection (CONTINUOUS)    │
│     - Starts camera processing          │
│     - Runs throughout entire mission    │
│     - Auto-updates database             │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  2. Navigate to Waypoint 1              │
│     - Nav2 handles path planning        │
│     - QR detection running in background│
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  3. Waypoint Reached!                   │
│     - Robot stops at waypoint           │
│     - Ready for inventory task          │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  4. Lift Actuator UP (5 seconds)        │
│     - Sends "up" command                │
│     - QR codes captured DURING lift     │
│     - Scans multiple shelf levels       │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  5. Wait at Top (50 seconds)            │
│     - Actuator fully raised             │
│     - QR detection captures all shelves │
│     - Database updates automatically    │
│     - Enhanced neon processing          │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  6. Lower Actuator DOWN (5 seconds)     │
│     - Sends "down" command              │
│     - Returns to travel position        │
│     - QR detection STILL running        │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  7. Move to Next Waypoint               │
│     - Nav2 navigates to waypoint 2      │
│     - Repeats steps 3-6 for each rack   │
│     - QR detection NEVER stops          │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  MISSION COMPLETE                       │
│  - All waypoints visited                │
│  - All QR codes in database             │
│  - QR detection keeps running           │
│  - Press Ctrl+C to stop                 │
└─────────────────────────────────────────┘
```

### Step 2: Monitor Progress

```bash
# Terminal 2: Monitor waypoint status
ros2 topic echo /waypoint_follower/status

# Terminal 3: Monitor QR detections
ros2 topic echo /qr_detections

# Terminal 4: Monitor actuator status
ros2 topic echo /actuator/status
```

### Step 3: View in RViz

```bash
ros2 launch nav2_bringup rviz_launch.py
```

You'll see:
- Robot position
- Current navigation goal
- Path to goal
- Map

---

## 🎛️ Configuration

### Scan Duration

How long to scan QR codes at each waypoint (default: 50 seconds)

```bash
ros2 launch mecanum_hardware inventory_mission.launch.py \
    scan_duration:=60.0  # 60 seconds
```

### Actuator Timing

Modify in launch file if needed:
```python
'actuator_lift_time': 5.0,   # Time to lift up
'actuator_lower_time': 5.0,  # Time to lower
```

### Camera Device

```bash
ros2 launch mecanum_hardware inventory_mission.launch.py \
    camera_device:=/dev/video0
```

---

## 📊 Database Integration

The system automatically updates the MongoDB database with:
- Mission ID
- Rack ID from QR code
- Shelf ID from QR code  
- Item code
- Timestamp
- Confidence score

**No manual database operations needed!** The enhanced QR detection system auto-updates.

---

## 🔧 Manual Control (If Needed)

### Manual Actuator Control

```bash
# Lift up
ros2 topic pub /actuator/command std_msgs/String "data: 'up'"

# Lower down
ros2 topic pub /actuator/command std_msgs/String "data: 'down'"

# Stop
ros2 topic pub /actuator/command std_msgs/String "data: 'stop'"
```

### Emergency Stop

Press `Ctrl+C` in the launch terminal

---

## 📝 Complete Workflow Example

```bash
# ========== DAY 1: CREATE WAYPOINTS ==========

# 1. Launch waypoint creation mode
ros2 launch mecanum_hardware waypoint_creation.launch.py \
    map:=~/maps/warehouse.yaml \
    waypoints_output:=~/warehouse_waypoints.yaml

# 2. Drive around with joystick, press A at each rack location
# 3. Saved 10 waypoints to ~/warehouse_waypoints.yaml

# ========== DAY 2: RUN INVENTORY MISSION ==========

# 1. Launch autonomous mission
ros2 launch mecanum_hardware inventory_mission.launch.py \
    map:=~/maps/warehouse.yaml \
    waypoints:=~/warehouse_waypoints.yaml \
    scan_duration:=50.0

# 2. Robot automatically:
#    - Visits all 10 waypoints
#    - Scans QR codes at each (50s each)
#    - Updates database
#    - Returns to start

# 3. Check results
#    Database has all scanned items!
```

---

## 🐛 Troubleshooting

### Robot Doesn't Move
```bash
# Check Nav2 status
ros2 node list | grep nav

# Check if robot is localized
ros2 topic echo /amcl_pose
```

### No QR Detections
```bash
# Check camera
ros2 topic echo /camera/image_raw

# Check QR detector
ros2 node info /qr_detector_enhanced
```

### Actuator Not Moving
```bash
# Check actuator node
ros2 node info /actuator_control

# Test manually
ros2 topic pub /actuator/command std_msgs/String "data: 'up'"
```

### Database Not Updating
```bash
# Check database node
ros2 node info /inventory_database

# Check MongoDB connection
# Ensure MongoDB is running or Atlas connection is set
```

---

## 📈 Mission Statistics

After mission completion, check:

```bash
# View exported mission data
cat /tmp/inventory_missions/MISSION_*.json | jq

# MongoDB (if using Atlas)
# Login to Atlas dashboard and view collections
```

---

## 🎯 Key Features

✅ **Fully Autonomous** - No human intervention after start  
✅ **Precise Navigation** - Nav2 + AMCL localization  
✅ **Timed Tasks** - Configurable scan duration  
✅ **Auto Database Update** - QR detections auto-saved  
✅ **Easy Waypoint Creation** - Just press a button  
✅ **Robust** - Handles navigation failures  
✅ **Flexible** - Configurable timing and parameters  
✅ **Cloud Storage** - MongoDB Atlas integration  

---

## 🚦 System Status Topics

Monitor these during operation:

| Topic | Purpose |
|-------|---------|
| `/waypoint_follower/status` | Current task status |
| `/qr_detections` | QR code detections |
| `/actuator/status` | Actuator state |
| `/actuator/command` | Actuator commands |
| `/cmd_vel` | Robot velocity commands |
| `/navigate_to_pose/_action/status` | Nav2 status |

---

**Ready to automate your warehouse! 🎉**
