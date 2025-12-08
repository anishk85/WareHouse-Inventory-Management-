# ROS2 Warehouse Rover - Web GUI Integration

Complete web-based control interface for the autonomous warehouse rover system.

## 🎯 Quick Start

### Terminal 1: Start ROS2 WebSocket Bridge
```bash
cd /root/ros2_ws
source install/setup.bash
ros2 run mecanum_hardware ros2_web_bridge.py
```

### Terminal 2: Start Web GUI
```bash
cd /root/ros2_ws/my-app/my-app
npm run dev
```

### Terminal 3: Test Connection (Optional)
```bash
cd /root/ros2_ws
source install/setup.bash
python3 src/mecanum_hardware/scripts/test_websocket_bridge.py
```

Then open: **http://localhost:3000**

---

## 📋 What's Been Created

### 1. ROS2 WebSocket Bridge (`ros2_web_bridge.py`)
- **Purpose**: Bidirectional communication between ROS2 and web browser
- **Port**: 9090
- **Features**:
  - Real-time topic data streaming
  - Command execution from GUI
  - Launch file management
  - System monitoring

**Subscribes to**:
- `/system_status` - Node and topic counts
- `/mission_status` - Current waypoint and task
- `/qr_detections` - QR code detection results
- `/actuator/status` - Lift actuator position
- `/scan` - LiDAR sensor data
- `/odom` - Robot odometry
- `/map` - SLAM map data
- `/battery_status` - Battery level
- `/goal_pose` - Navigation goal

**Publishes to**:
- `/cmd_vel` - Velocity commands
- `/actuator/control` - Actuator control
- `/qr_detection/enable` - QR detection toggle
- `/nav2/goal_pose` - Navigation goals

### 2. React Hook (`useROS2.ts`)
- **Purpose**: React interface for WebSocket communication
- **Features**:
  - Auto-reconnection
  - Type-safe message handling
  - Command helpers
  - State management

**Functions**:
```typescript
publishCmdVel(linear, angular)      // Send velocity command
controlActuator(command)             // Control lift actuator
enableQRDetection()                  // Start QR detection
disableQRDetection()                 // Stop QR detection
sendNavGoal(x, y, theta)            // Send navigation goal
cancelNavigation()                   // Cancel current navigation
launch(package, file, args)         // Start launch file
stopLaunch(launchId)                // Stop launch file
```

### 3. Dashboard Component (`WarehouseRoverDashboard.tsx`)
- **Purpose**: Main control interface
- **Sections**:
  - System status (nodes, topics, uptime)
  - Mission status (waypoint, task, progress)
  - Launch control (11 configurations)
  - QR detection results
  - Sensor data display
  - Quick controls

### 4. Launch Configurations (`launch-configs.ts`)
Pre-configured launch files organized by category:

**System**:
- Hardware Bringup
- Robot State Publisher

**Mapping**:
- Cartographer SLAM
- Mapping with Teleop

**Navigation**:
- Nav2 Localization
- Nav2 with Map

**Inventory**:
- Waypoint Creation
- Waypoint Follower
- Nav2 Waypoint Follower

**Testing**:
- Test Laser Sensor
- Test QR View

---

## 🏗️ Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                         Web Browser                          │
│  ┌────────────────────────────────────────────────────────┐ │
│  │        WarehouseRoverDashboard Component              │ │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐            │ │
│  │  │ System   │  │ Launch   │  │ QR       │            │ │
│  │  │ Status   │  │ Control  │  │ Results  │            │ │
│  │  └──────────┘  └──────────┘  └──────────┘            │ │
│  │                      ▲                                │ │
│  │                      │ useROS2 Hook                   │ │
│  │                      ▼                                │ │
│  │              WebSocket Connection                     │ │
│  └──────────────────┬──────────────────────────────────┘ │
└─────────────────────┼─────────────────────────────────────┘
                      │
                      │ ws://localhost:9090
                      │
┌─────────────────────┼─────────────────────────────────────┐
│                     ▼                                     │
│         ROS2 WebSocket Bridge (Python)                    │
│  ┌─────────────────────────────────────────────────────┐ │
│  │  WebSocket Server                                   │ │
│  │  ├─ Handle connections                              │ │
│  │  ├─ Route messages                                  │ │
│  │  └─ Manage launch processes                         │ │
│  └────────┬────────────────────────────────────────────┘ │
│           │                                               │
│  ┌────────┴─────────────────────────────────────────┐    │
│  │         ROS2 Topic Subscriptions                 │    │
│  │  /system_status  /qr_detections  /actuator/status│    │
│  │  /mission_status  /scan  /odom  /map  /battery   │    │
│  └──────────────────────────────────────────────────┘    │
│           │                                               │
│  ┌────────┴─────────────────────────────────────────┐    │
│  │         ROS2 Topic Publishers                    │    │
│  │  /cmd_vel  /actuator/control  /qr_detection/enable│   │
│  │  /nav2/goal_pose  /nav2/cancel_goal              │    │
│  └──────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────┘
```

---

## 🔧 Configuration

### Environment Variables

**`.env.local`** (Frontend):
```bash
NEXT_PUBLIC_WS_URL=ws://localhost:9090
```

For remote access:
```bash
NEXT_PUBLIC_WS_URL=ws://192.168.1.100:9090  # Replace with robot's IP
```

---

## 🧪 Testing

### 1. Test WebSocket Bridge
```bash
cd /root/ros2_ws
source install/setup.bash
python3 src/mecanum_hardware/scripts/test_websocket_bridge.py
```

Expected output:
```
🔌 Connecting to ROS2 WebSocket Bridge...
✅ Connected successfully!
📊 Waiting for messages from bridge...
📨 Message #1:
   Type: system_status
   Active Nodes: 12
   Active Topics: 45
   Uptime: 10.5s
...
✅ Test completed successfully!
```

### 2. Test Frontend Connection
1. Open browser: http://localhost:3000
2. Check connection badge (top right) - should be green "Connected"
3. Open browser console (F12)
4. Should see: `WebSocket connected to ws://localhost:9090`

### 3. Test Launch Control
1. Click "System" tab
2. Click "Launch" on "Hardware Bringup"
3. Verify launch status changes to "Running"
4. Check Terminal 1 for ROS2 nodes starting

### 4. Test Actuator Control
1. Go to "Quick Controls" section
2. Click "Lift Up"
3. Verify in terminal: `ros2 topic echo /actuator/control`

### 5. Test QR Detection
1. Click "Enable QR Detection"
2. Check QR detection results section
3. Verify in terminal: `ros2 topic echo /qr_detections`

---

## 📁 File Locations

```
/root/ros2_ws/
├── src/mecanum_hardware/
│   ├── scripts/
│   │   ├── ros2_web_bridge.py              # WebSocket bridge server
│   │   ├── test_websocket_bridge.py        # Test script
│   │   ├── waypoint_follower_node.py       # Manual waypoint navigation
│   │   └── nav2_waypoint_follower.py       # Nav2 waypoint navigation
│   └── launch/
│       └── ... (launch files)
│
├── my-app/my-app/
│   ├── .env.local                          # Environment config
│   ├── hooks/
│   │   └── useROS2.ts                      # React WebSocket hook
│   ├── lib/
│   │   └── launch-configs.ts               # Launch definitions
│   └── components/
│       ├── WarehouseRoverDashboard.tsx     # Main dashboard
│       └── ui/
│           ├── badge.tsx                   # UI components
│           ├── tabs.tsx
│           ├── scroll-area.tsx
│           ├── alert.tsx
│           └── progress.tsx
│
├── GUI_INTEGRATION_GUIDE.md               # Detailed guide
└── GUI_SETUP_README.md                    # This file
```

---

## 🚀 Features

### Real-Time Monitoring
- ✅ System status (nodes, topics, uptime)
- ✅ Mission progress tracking
- ✅ QR detection results
- ✅ Sensor data (LiDAR, odometry, battery)
- ✅ Actuator position

### Remote Control
- ✅ Launch file management (start/stop)
- ✅ Velocity control (cmd_vel)
- ✅ Actuator control (lift/lower)
- ✅ QR detection toggle
- ✅ Navigation goals

### Dashboard UI
- ✅ Organized by categories (System, Mapping, Navigation, Inventory, Testing)
- ✅ Real-time connection status
- ✅ Color-coded status indicators
- ✅ Responsive layout
- ✅ Quick action buttons

---

## 🛠️ Customization

### Add New Launch Configuration
Edit `/root/ros2_ws/my-app/my-app/lib/launch-configs.ts`:

```typescript
{
  id: 'my-new-launch',
  name: 'My New Launch',
  description: 'Description of what it does',
  category: 'system',
  launchFile: 'my_launch.launch.py',
  params: [],
  icon: '🚀'
}
```

### Add New ROS2 Topic
Edit `/root/ros2_ws/src/mecanum_hardware/scripts/ros2_web_bridge.py`:

```python
# Add subscription
self.my_subscription = self.create_subscription(
    MyMessageType,
    '/my_topic',
    self.my_topic_callback,
    10
)

# Add callback
def my_topic_callback(self, msg):
    data = {
        'type': 'my_topic',
        'data': {'field': msg.field}
    }
    asyncio.run(self.broadcast_message(data))
```

### Modify Dashboard Layout
Edit `/root/ros2_ws/my-app/my-app/components/WarehouseRoverDashboard.tsx` to change sections, colors, or add new features.

---

## 🐛 Troubleshooting

### WebSocket Connection Failed

**Symptoms**: Dashboard shows "Disconnected"

**Solutions**:
1. Check bridge is running: `ps aux | grep ros2_web_bridge`
2. Check port: `netstat -tulpn | grep 9090`
3. Verify `.env.local` has correct URL
4. Check browser console for errors

### Launch Not Starting

**Symptoms**: Launch button does nothing

**Solutions**:
1. Verify ROS2 workspace sourced in bridge terminal
2. Check launch file exists: `ls src/mecanum_hardware/launch/`
3. Check bridge logs in Terminal 1
4. Verify launch file path in `launch-configs.ts`

### No Data Displayed

**Symptoms**: Dashboard shows "No data"

**Solutions**:
1. Check ROS2 nodes publishing: `ros2 topic list`
2. Test topic: `ros2 topic echo /qr_detections`
3. Verify bridge subscriptions in Terminal 1
4. Check message types match

---

## 📚 Additional Documentation

- **Detailed Integration Guide**: `GUI_INTEGRATION_GUIDE.md`
- **Inventory System**: `INVENTORY_SYSTEM_GUIDE.md`
- **Nav2 Comparison**: `NAV2_WAYPOINT_COMPARISON.md`

---

## 🎓 Learning Resources

### WebSocket Communication
- WebSocket protocol: https://websockets.spec.whatwg.org/
- Python websockets: https://websockets.readthedocs.io/

### ROS2 Navigation
- Nav2 documentation: https://navigation.ros.org/
- Waypoint following: https://navigation.ros.org/behavior_trees/

### React Hooks
- React hooks: https://react.dev/reference/react
- Custom hooks: https://react.dev/learn/reusing-logic-with-custom-hooks

---

## 💡 Tips

1. **Keep bridge terminal visible**: Watch for real-time logs and errors
2. **Use browser console**: Check for WebSocket connection issues
3. **Test incrementally**: Start with basic connection, then add features
4. **Monitor ROS2 topics**: Use `ros2 topic echo` to verify data flow
5. **Check network**: For remote access, ensure ports are open (9090, 3000)

---

## 🔒 Security Notes

For production deployment:

1. **Use WSS**: Secure WebSocket (wss://) instead of ws://
2. **Add authentication**: Token-based auth for WebSocket connection
3. **Rate limiting**: Prevent command flooding
4. **Input validation**: Sanitize all commands from GUI
5. **HTTPS**: Use HTTPS for Next.js frontend

---

## 🤝 Contributing

To add new features:

1. **Backend**: Modify `ros2_web_bridge.py` to handle new topics/services
2. **Hook**: Update `useROS2.ts` to expose new functions
3. **UI**: Add components to `WarehouseRoverDashboard.tsx`
4. **Config**: Add launch configurations to `launch-configs.ts`
5. **Test**: Update `test_websocket_bridge.py` with new tests

---

## ✅ Next Steps

1. ✅ **Installed UI components** (badge, tabs, scroll-area, alert, progress)
2. ✅ **Created WebSocket bridge** (ros2_web_bridge.py)
3. ✅ **Created React hook** (useROS2.ts)
4. ✅ **Created dashboard** (WarehouseRoverDashboard.tsx)
5. ✅ **Configured environment** (.env.local)
6. ✅ **Created test script** (test_websocket_bridge.py)
7. 🔄 **Test connection**: Start bridge and frontend, verify WebSocket
8. 🔄 **Test features**: Launch configs, QR detection, actuator control
9. 🔄 **Integration**: Merge with existing dashboard or use standalone
10. 🔄 **Deploy**: Configure for production with reverse proxy

---

## 📞 Support

If you encounter issues:

1. **Check logs**: Terminal 1 (bridge), browser console (frontend)
2. **Test topics**: Use ROS2 CLI tools (`ros2 topic echo`, `ros2 node list`)
3. **Run test script**: `python3 test_websocket_bridge.py`
4. **Verify configuration**: Check `.env.local` and launch file paths
5. **Network**: Ensure ports 9090 and 3000 are accessible

---

**Happy coding! 🚀🤖**
