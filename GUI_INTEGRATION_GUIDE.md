# Web GUI Setup and Integration Guide

## Overview
This guide explains how to set up and test the web GUI integration with the ROS2 warehouse rover backend.

## Architecture

```
┌─────────────────┐         WebSocket (port 9090)        ┌─────────────────┐
│   Next.js App   │ ◄──────────────────────────────────► │  ROS2 Bridge    │
│  (React UI)     │                                       │  (Python)       │
└─────────────────┘                                       └─────────────────┘
        │                                                          │
        │                                                          │
        ▼                                                          ▼
   User Browser                                          ROS2 Topics/Services
   - View data                                           - /cmd_vel
   - Control robot                                       - /qr_detections
   - Launch configs                                      - /actuator/control
   - Monitor status                                      - /nav2 actions
                                                         - etc.
```

## Components Created

### 1. ROS2 WebSocket Bridge (`ros2_web_bridge.py`)
**Location**: `/root/ros2_ws/src/mecanum_hardware/scripts/ros2_web_bridge.py`

**Features**:
- WebSocket server on port 9090
- Subscribes to 10+ ROS2 topics
- Publishes commands from web GUI to ROS2
- Manages launch file execution
- Real-time status updates

**ROS2 Topics Subscribed**:
- `/system_status` - System health
- `/mission_status` - Current mission state
- `/qr_detections` - QR code detections
- `/actuator/status` - Actuator position
- `/scan` - LiDAR data
- `/odom` - Odometry
- `/map` - Occupancy grid map
- `/cmd_vel` - Velocity commands (echo)
- `/battery_status` - Battery level
- `/goal_pose` - Navigation goal

**Commands from Web GUI**:
- `cmd_vel` - Send velocity commands
- `actuator_control` - Control lift actuator
- `qr_control` - Enable/disable QR detection
- `nav_goal` - Send navigation goal
- `cancel_nav` - Cancel current navigation
- `launch` - Start ROS2 launch file
- `stop_launch` - Stop running launch

### 2. React Hook (`useROS2.ts`)
**Location**: `/root/ros2_ws/my-app/my-app/hooks/useROS2.ts`

**Features**:
- Manages WebSocket connection
- Auto-reconnection on disconnect
- Type-safe message handling
- Command publishing functions

**Exported Functions**:
```typescript
const {
  connected,
  systemStatus,
  missionStatus,
  qrDetections,
  actuatorStatus,
  sensorData,
  publishCmdVel,
  controlActuator,
  enableQRDetection,
  disableQRDetection,
  sendNavGoal,
  cancelNavigation,
  launch,
  stopLaunch,
} = useROS2();
```

### 3. Dashboard Component (`WarehouseRoverDashboard.tsx`)
**Location**: `/root/ros2_ws/my-app/my-app/components/WarehouseRoverDashboard.tsx`

**Features**:
- Real-time system status display
- Launch configuration management
- QR detection results
- Sensor data visualization
- Quick controls for actuator and QR
- Mission status tracking

### 4. Launch Configurations (`launch-configs.ts`)
**Location**: `/root/ros2_ws/my-app/my-app/lib/launch-configs.ts`

**Available Launches** (11 total):
1. **System**:
   - Hardware bringup
   - Robot state publisher

2. **Mapping**:
   - Cartographer SLAM
   - Mapping with teleop

3. **Navigation**:
   - Nav2 localization
   - Nav2 with map

4. **Inventory**:
   - Waypoint creation
   - Waypoint follower
   - Nav2 waypoint follower

5. **Testing**:
   - Test laser sensor
   - Test QR view

## Setup Instructions

### Step 1: Build ROS2 Workspace
```bash
cd /root/ros2_ws
colcon build --packages-select mecanum_hardware
source install/setup.bash
```

### Step 2: Start ROS2 WebSocket Bridge
```bash
# Terminal 1
cd /root/ros2_ws
source install/setup.bash
ros2 run mecanum_hardware ros2_web_bridge.py
```

Expected output:
```
[INFO] [ros2_web_bridge]: WebSocket server starting on port 9090
[INFO] [ros2_web_bridge]: ROS2 node initialized
[INFO] [ros2_web_bridge]: Subscribed to 10 topics
[INFO] [ros2_web_bridge]: Server ready for connections
```

### Step 3: Start Next.js Frontend
```bash
# Terminal 2
cd /root/ros2_ws/my-app/my-app
npm run dev
```

Expected output:
```
  ▲ Next.js 16.0.7
  - Local:        http://localhost:3000
  - Environments: .env.local

 ✓ Starting...
 ✓ Ready in 2.3s
```

### Step 4: Access Dashboard
Open browser: `http://localhost:3000`

You should see:
- **Connection Status**: Green badge "Connected" (top right)
- **System Status**: Active nodes, topics, bridge uptime
- **Mission Status**: Current waypoint, task progress
- **Launch Control**: 11 launch configurations organized by category
- **QR Detections**: Real-time QR code results
- **Sensor Data**: Battery, LiDAR, odometry

## Testing the Integration

### Test 1: WebSocket Connection
1. Open browser console (F12)
2. Should see: `WebSocket connected to ws://localhost:9090`
3. Dashboard should show green "Connected" badge

### Test 2: Launch a Configuration
1. In dashboard, click "System" tab
2. Click "Launch" on "Hardware Bringup"
3. Should see launch status change to "Running"
4. Check Terminal 1 for ROS2 nodes starting

### Test 3: Control Actuator
1. In "Quick Controls" section
2. Click "Lift Up" button
3. Should see actuator status change
4. Verify in ROS2: `ros2 topic echo /actuator/control`

### Test 4: QR Detection
1. Click "Enable QR Detection" button
2. Should see QR detections appear in real-time
3. Verify in ROS2: `ros2 topic echo /qr_detections`

### Test 5: Send Velocity Command
1. Use keyboard controls or joystick UI (if implemented)
2. Should see robot move
3. Verify in ROS2: `ros2 topic echo /cmd_vel`

## Troubleshooting

### WebSocket Connection Failed
**Problem**: Dashboard shows "Disconnected" status

**Solutions**:
1. Check if bridge is running: `ps aux | grep ros2_web_bridge`
2. Check port 9090: `netstat -tulpn | grep 9090`
3. Verify `.env.local`: `NEXT_PUBLIC_WS_URL=ws://localhost:9090`
4. Check browser console for errors

### Launch Not Starting
**Problem**: Launch button does nothing

**Solutions**:
1. Verify launch file exists: `ls /root/ros2_ws/src/mecanum_hardware/launch/`
2. Check ROS2 workspace sourced in bridge terminal
3. Check bridge logs in Terminal 1
4. Verify launch file path in `launch-configs.ts`

### No Data Displayed
**Problem**: Dashboard shows "No data" or empty fields

**Solutions**:
1. Check if ROS2 nodes are publishing: `ros2 topic list`
2. Verify bridge subscriptions: Check Terminal 1 logs
3. Test topic directly: `ros2 topic echo /qr_detections`
4. Check message types match in bridge code

### QR Detection Not Working
**Problem**: No QR codes detected even when visible

**Solutions**:
1. Enable QR detection: Click "Enable QR Detection" button
2. Verify camera node running: `ros2 node list | grep camera`
3. Check QR detection node: `ros2 node list | grep qr`
4. Test with test script: `ros2 run mecanum_hardware test_view.py`

## Environment Variables

### Frontend (`.env.local`)
```bash
NEXT_PUBLIC_WS_URL=ws://localhost:9090  # WebSocket bridge URL
```

### Production Deployment
For remote access, update:
```bash
NEXT_PUBLIC_WS_URL=wss://your-robot-ip:9090  # Use WSS for secure connection
```

## File Structure

```
/root/ros2_ws/
├── src/mecanum_hardware/
│   ├── scripts/
│   │   ├── ros2_web_bridge.py          # WebSocket bridge (476 lines)
│   │   ├── waypoint_follower_node.py   # Manual waypoint navigation
│   │   └── nav2_waypoint_follower.py   # Nav2 waypoint navigation
│   └── launch/
│       ├── bringup.launch.py
│       ├── nav2_with_map.launch.py
│       └── ... (other launch files)
│
└── my-app/my-app/
    ├── .env.local                      # Environment config
    ├── hooks/
    │   └── useROS2.ts                  # React WebSocket hook (239 lines)
    ├── lib/
    │   └── launch-configs.ts           # Launch configuration definitions
    └── components/
        ├── WarehouseRoverDashboard.tsx # Main dashboard (333 lines)
        └── ui/
            ├── badge.tsx               # shadcn/ui components
            ├── tabs.tsx
            ├── scroll-area.tsx
            ├── alert.tsx
            └── progress.tsx
```

## Key Features

### 1. Real-Time Updates
- WebSocket provides ~30Hz update rate
- No polling required
- Low latency (<50ms)

### 2. Launch Management
- Start/stop any ROS2 launch file from GUI
- Track launch status
- View launch logs

### 3. QR Code Tracking
- Real-time QR detection results
- Displays QR content and confidence
- Timestamp for each detection

### 4. Navigation Control
- Send goal poses
- Cancel navigation
- Monitor waypoint progress

### 5. System Monitoring
- Active nodes count
- Active topics count
- Bridge uptime
- Connection status

## Integration with Existing System

The new dashboard (`WarehouseRoverDashboard.tsx`) can:

1. **Replace existing dashboard**: Update `/root/ros2_ws/my-app/my-app/components/pages/dashboard.tsx`
2. **Coexist as separate page**: Add new route in navigation
3. **Merge features**: Combine 3D rover visualization from existing dashboard with new ROS2 controls

### Example: Replace Existing Dashboard
```typescript
// /root/ros2_ws/my-app/my-app/components/pages/dashboard.tsx
import WarehouseRoverDashboard from '@/components/WarehouseRoverDashboard';

export default function Dashboard() {
  return <WarehouseRoverDashboard />;
}
```

## Next Steps

1. **Test Connection**: Start bridge and frontend, verify WebSocket connection
2. **Launch Hardware**: Use GUI to launch hardware bringup
3. **Test QR Detection**: Enable QR detection and verify results display
4. **Test Navigation**: Send a navigation goal and monitor status
5. **Customize UI**: Modify dashboard layout, colors, or add features
6. **Add 3D Visualization**: Integrate Three.js rover model from existing dashboard
7. **Deploy**: Configure for production with reverse proxy and SSL

## API Reference

### WebSocket Message Format

#### Client → Server (Commands)
```json
{
  "type": "cmd_vel",
  "data": {
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
  }
}

{
  "type": "actuator_control",
  "data": {"command": "lift_up"}
}

{
  "type": "launch",
  "data": {
    "package": "mecanum_hardware",
    "launch_file": "bringup.launch.py",
    "arguments": []
  }
}
```

#### Server → Client (Updates)
```json
{
  "type": "system_status",
  "data": {
    "active_nodes": 12,
    "active_topics": 45,
    "bridge_uptime": 123.45
  }
}

{
  "type": "qr_detections",
  "data": {
    "detections": [
      {
        "content": "RACK-A-01",
        "confidence": 0.95,
        "timestamp": 1234567890.123
      }
    ]
  }
}
```

## Maintenance

### Update Launch Configurations
Edit `/root/ros2_ws/my-app/my-app/lib/launch-configs.ts` to add new launch files.

### Modify Dashboard Layout
Edit `/root/ros2_ws/my-app/my-app/components/WarehouseRoverDashboard.tsx` to change UI.

### Add New ROS2 Topics
Edit `/root/ros2_ws/src/mecanum_hardware/scripts/ros2_web_bridge.py` to subscribe to additional topics.

### Debug WebSocket
Enable verbose logging in bridge:
```python
# In ros2_web_bridge.py
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

## Performance Tips

1. **Reduce update rate**: If UI is laggy, throttle message publishing in bridge
2. **Filter topics**: Only subscribe to topics you need
3. **Compress large messages**: Use message_filters for sensor data
4. **Use production build**: `npm run build && npm start` for better performance

## Security Considerations

1. **Authentication**: Add token-based auth to WebSocket
2. **SSL/TLS**: Use WSS for encrypted communication
3. **Input validation**: Sanitize commands from web GUI
4. **Rate limiting**: Prevent command flooding from malicious clients
5. **CORS**: Configure proper CORS headers for production

## Support

For issues or questions:
- Check ROS2 logs: `ros2 topic list`, `ros2 node list`
- Check browser console: F12 → Console
- Review bridge logs in Terminal 1
- Test individual components with ROS2 CLI tools
