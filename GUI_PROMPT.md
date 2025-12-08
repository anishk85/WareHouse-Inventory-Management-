# 🤖 AI Agent Prompt: Warehouse Rover Control GUI

## Project Overview
Create a modern, futuristic web-based GUI for controlling an autonomous warehouse inventory robot. The system uses ROS2, Nav2 navigation, QR detection, and database management. The interface should have a dark, robotic/cyberpunk theme similar to the reference: https://github.com/Team-Deimos-IIT-Mandi/Robotic-Arm-Gui

---

## 🎨 Design Requirements

### Theme & Aesthetics
- **Dark theme** with neon accents (cyan, purple, green)
- **Futuristic/Cyberpunk** aesthetic
- **Robotic feel** with geometric patterns
- **Glassmorphism** effects for modern look
- **Smooth animations** and transitions
- **Responsive design** (works on desktop, tablet, mobile)
- Color palette:
  - Primary: `#0a0e27` (dark blue-black background)
  - Secondary: `#1a1f3a` (card backgrounds)
  - Accent 1: `#00d9ff` (cyan - for active states)
  - Accent 2: `#bd00ff` (purple - for highlights)
  - Success: `#00ff88` (green - for success states)
  - Warning: `#ffaa00` (orange - for warnings)
  - Error: `#ff0055` (red - for errors)

### Typography
- Primary font: `Orbitron` or `Rajdhani` (futuristic)
- Monospace font: `Fira Code` or `JetBrains Mono` (for code/data)

---

## 📋 System Architecture

### Backend (ROS2 Bridge)
Create a **Python Flask/FastAPI** backend that:
1. Communicates with ROS2 nodes via `rclpy`
2. Launches ROS2 launch files using subprocess
3. Subscribes to ROS2 topics and streams data via WebSockets
4. Provides REST API for commands
5. Monitors system status

### Frontend (React/Vue/Svelte)
Build a **modern SPA** with:
1. Real-time data visualization
2. Interactive controls
3. Live video feed from robot camera
4. Map visualization with robot position
5. Database viewer for inventory

---

## 🎯 Core Features & Pages

### 1. Dashboard (Home Page)
**Layout:**
```
┌────────────────────────────────────────────────────────────┐
│  Header: Logo | Robot Status | Connection Status           │
├────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │ System Stats │  │ Quick Launch │  │ Live Camera  │    │
│  │ • CPU: 45%   │  │ [Hardware]   │  │              │    │
│  │ • RAM: 2.1GB │  │ [Mapping]    │  │   [VIDEO]    │    │
│  │ • Disk: 60%  │  │ [Navigation] │  │              │    │
│  └──────────────┘  └──────────────┘  └──────────────┘    │
│                                                            │
│  ┌────────────────────────────────────────────────────┐   │
│  │ Recent Activity Log                                │   │
│  │ [12:30:45] Mission started...                      │   │
│  │ [12:31:20] Waypoint 1 reached ✓                   │   │
│  │ [12:32:15] QR detected: R01-S1-ITEM001            │   │
│  └────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────┘
```

**Features:**
- Real-time system metrics (CPU, RAM, network)
- ROS2 node status indicators (green/red dots)
- Quick launch buttons with loading states
- Activity feed with color-coded messages
- Robot battery level gauge (circular progress)
- Live camera feed with overlay info

---

### 2. Launch Control Page
**Layout:**
```
┌────────────────────────────────────────────────────────────┐
│  LAUNCH CONTROL                                             │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌─────────────────────────────────────────────────┐      │
│  │ 🤖 HARDWARE LAYER                                │      │
│  │ ┌───────────────────────────────────────────┐   │      │
│  │ │ Launch Hardware                            │   │      │
│  │ │ • Motors, Controllers, Sensors             │   │      │
│  │ │                                            │   │      │
│  │ │ Status: ⚫ Not Running                     │   │      │
│  │ │                                            │   │      │
│  │ │ [▶ Launch]  [⏹ Stop]  [🔄 Restart]       │   │      │
│  │ └───────────────────────────────────────────┘   │      │
│  │                                                  │      │
│  │ Parameters:                                      │      │
│  │ ├─ use_sim: false  [toggle]                    │      │
│  │ ├─ serial_port: /dev/ttyUSB0                   │      │
│  │ └─ lidar_port: /dev/ttyUSB2                    │      │
│  │                                                  │      │
│  │ Output Console:                                  │      │
│  │ ┌────────────────────────────────────────────┐ │      │
│  │ │ [INFO] Hardware started successfully       │ │      │
│  │ │ [INFO] Controllers active: 4/4             │ │      │
│  │ └────────────────────────────────────────────┘ │      │
│  └─────────────────────────────────────────────────┘      │
│                                                            │
│  ┌─────────────────────────────────────────────────┐      │
│  │ 🗺️  MAPPING MODE                                 │      │
│  │ ... (similar card structure)                    │      │
│  └─────────────────────────────────────────────────┘      │
│                                                            │
│  ┌─────────────────────────────────────────────────┐      │
│  │ 🧭 NAVIGATION MODE                               │      │
│  │ ... (similar card structure)                    │      │
│  └─────────────────────────────────────────────────┘      │
│                                                            │
│  ┌─────────────────────────────────────────────────┐      │
│  │ 📦 INVENTORY MISSION                             │      │
│  │ ├─ Map File: [Browse...]                        │      │
│  │ ├─ Waypoints: [Browse...]                       │      │
│  │ ├─ Scan Duration: [50] seconds                  │      │
│  │ └─ Use Nav2 Follower: [✓]                      │      │
│  │ [▶ Launch Mission]                              │      │
│  └─────────────────────────────────────────────────┘      │
└────────────────────────────────────────────────────────────┘
```

**Features:**
- Collapsible cards for each launch file
- Real-time parameter editing
- Live console output (scrollable)
- One-click launch/stop buttons
- Process status indicators
- Save/load parameter presets
- Launch history

---

### 3. Navigation Control Page
**Layout:**
```
┌────────────────────────────────────────────────────────────┐
│  NAVIGATION CONTROL                                         │
├──────────────────┬─────────────────────────────────────────┤
│                  │                                         │
│  Map Viewer      │  Robot Controls                        │
│  ┌────────────┐  │  ┌──────────────────────────────────┐ │
│  │            │  │  │ Set Initial Pose                  │ │
│  │            │  │  │ [📍 Click mode: Initial Pose]    │ │
│  │   [MAP]    │  │  └──────────────────────────────────┘ │
│  │            │  │                                         │
│  │    🤖       │  │  ┌──────────────────────────────────┐ │
│  │            │  │  │ Send Navigation Goal              │ │
│  └────────────┘  │  │ [🎯 Click mode: Nav Goal]        │ │
│                  │  └──────────────────────────────────┘ │
│  Layers:         │                                         │
│  [✓] Map         │  ┌──────────────────────────────────┐ │
│  [✓] Robot       │  │ Waypoint Management               │ │
│  [✓] Path        │  │ • Waypoints: 3                    │ │
│  [✓] Obstacles   │  │ [View] [Edit] [Clear]            │ │
│  [ ] Costmap     │  └──────────────────────────────────┘ │
│                  │                                         │
│  Tools:          │  ┌──────────────────────────────────┐ │
│  [📏] Measure    │  │ Teleop Control                    │ │
│  [🎨] Draw       │  │   [↑]                             │ │
│  [🗑️] Clear      │  │ [←][↓][→]                        │ │
│                  │  │                                    │ │
│                  │  │ Speed: [=====>---] 50%           │ │
│                  │  └──────────────────────────────────┘ │
└──────────────────┴─────────────────────────────────────────┘
```

**Features:**
- Interactive map display (using ROS2 Web Bridge)
- Click to set initial pose or navigation goals
- Real-time robot position visualization
- Path planning visualization
- Obstacle detection overlay
- Teleop joystick/keyboard controls
- Speed adjustment slider
- Waypoint marker management

---

### 4. Waypoint Creator Page
**Layout:**
```
┌────────────────────────────────────────────────────────────┐
│  WAYPOINT CREATOR                                           │
├──────────────────┬─────────────────────────────────────────┤
│                  │                                         │
│  Map View        │  Waypoint List                         │
│  ┌────────────┐  │  ┌──────────────────────────────────┐ │
│  │            │  │  │ 1. Rack A1                        │ │
│  │   🗺️       │  │  │    X: 2.5, Y: 1.3, θ: 0.0       │ │
│  │    📍       │  │  │    [Edit] [Delete] [Go To]       │ │
│  │     📍      │  │  ├──────────────────────────────────┤ │
│  │            │  │  │ 2. Rack A2                        │ │
│  └────────────┘  │  │    X: 5.1, Y: 3.8, θ: 1.57       │ │
│                  │  │    [Edit] [Delete] [Go To]       │ │
│  Mode:           │  ├──────────────────────────────────┤ │
│  ⚫ Add Waypoint │  │ 3. Rack B1                        │ │
│  ⚪ Edit Waypoint│  │    X: 7.2, Y: 2.1, θ: 3.14       │ │
│  ⚪ Delete WP    │  │    [Edit] [Delete] [Go To]       │ │
│                  │  └──────────────────────────────────┘ │
│  Current Pose:   │                                         │
│  X: 0.0          │  ┌──────────────────────────────────┐ │
│  Y: 0.0          │  │ Actions                           │ │
│  θ: 0.0          │  │ [💾 Save to File]                │ │
│                  │  │ [📂 Load from File]              │ │
│  [🎮 Start       │  │ [📤 Export as JSON]              │ │
│   Teleop Mode]   │  │ [🚀 Start Mission]               │ │
│                  │  └──────────────────────────────────┘ │
└──────────────────┴─────────────────────────────────────────┘
```

**Features:**
- Click on map to add waypoints
- Drag waypoints to reposition
- Teleop mode with save button
- Waypoint sequence reordering (drag & drop)
- Save/load waypoint files
- Test navigation to individual waypoints

---

### 5. QR Detection & Inventory Page
**Layout:**
```
┌────────────────────────────────────────────────────────────┐
│  INVENTORY MANAGEMENT                                       │
├──────────────────┬─────────────────────────────────────────┤
│                  │                                         │
│  Camera Feed     │  Detection Stats                       │
│  ┌────────────┐  │  ┌──────────────────────────────────┐ │
│  │            │  │  │ Current Mission                   │ │
│  │  📷 LIVE   │  │  │ ID: MISSION_20251207_143052      │ │
│  │            │  │  │ Status: 🟢 Active                │ │
│  │   [QR]     │  │  │ Duration: 00:15:32               │ │
│  │            │  │  │                                    │ │
│  └────────────┘  │  │ Detections Today: 247            │ │
│                  │  │ Success Rate: 94.2%              │ │
│  QR Enable:      │  │ Unique Items: 68                 │ │
│  [🟢 ON] [OFF]  │  │ Unique Racks: 12                 │ │
│                  │  └──────────────────────────────────┘ │
│  Last Detection: │                                         │
│  R01-S2-ITEM045  │  Recent Detections                     │
│  Confidence: 95% │  ┌──────────────────────────────────┐ │
│                  │  │ Time    │ Rack │ Item     │ Conf  │ │
│                  │  ├─────────┼──────┼──────────┼───────┤ │
│                  │  │ 14:30:15│ R01  │ ITEM045  │ 95%   │ │
│                  │  │ 14:29:50│ R01  │ ITEM044  │ 92%   │ │
│                  │  │ 14:29:30│ R01  │ ITEM043  │ 89%   │ │
│                  │  │ 14:29:10│ R01  │ ITEM042  │ 97%   │ │
│                  │  └──────────────────────────────────┘ │
│                  │                                         │
│                  │  [📊 View Full Database]               │
│                  │  [📥 Export CSV]                       │
│                  │  [🔍 Search Items]                     │
└──────────────────┴─────────────────────────────────────────┘
```

**Features:**
- Live camera feed with QR overlay
- Real-time detection notifications
- Detection statistics dashboard
- Searchable inventory table
- Filter by rack, date, confidence
- Export data (CSV, JSON, Excel)
- Barcode scanner sound effects

---

### 6. Database Viewer Page
**Layout:**
```
┌────────────────────────────────────────────────────────────┐
│  DATABASE VIEWER                                            │
├────────────────────────────────────────────────────────────┤
│  Filters: [Rack: All ▾] [Date: Today ▾] [Min Conf: 80%]  │
│  Search: [___________________] [🔍]                        │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────────────────────────────────────────────┐ │
│  │ Mission │ Time      │ Rack │ Shelf │ Item     │ Conf │ │
│  ├─────────┼───────────┼──────┼───────┼──────────┼──────┤ │
│  │ M_001   │ 14:30:15  │ R01  │ S1    │ ITEM045  │ 95%  │ │
│  │ M_001   │ 14:29:50  │ R01  │ S1    │ ITEM044  │ 92%  │ │
│  │ M_001   │ 14:29:30  │ R01  │ S2    │ ITEM043  │ 89%  │ │
│  │ M_001   │ 14:29:10  │ R01  │ S2    │ ITEM042  │ 97%  │ │
│  │ ...     │ ...       │ ...  │ ...   │ ...      │ ...  │ │
│  └──────────────────────────────────────────────────────┘ │
│                                                            │
│  Showing 1-50 of 247 entries   [< 1 2 3 ... 5 >]         │
│                                                            │
│  ┌──────────────────────────────────────────────────────┐ │
│  │ Statistics by Rack                                    │ │
│  │ [Bar Chart showing detections per rack]              │ │
│  └──────────────────────────────────────────────────────┘ │
│                                                            │
│  [📥 Export Selection] [🗑️ Clear Old Data]              │
└────────────────────────────────────────────────────────────┘
```

**Features:**
- Paginated table with sorting
- Advanced filtering options
- Charts and analytics
- Bulk operations
- Data export options

---

### 7. System Monitor Page
**Layout:**
```
┌────────────────────────────────────────────────────────────┐
│  SYSTEM MONITOR                                             │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌─────────────────┐  ┌─────────────────┐  ┌───────────┐ │
│  │ CPU Usage       │  │ Memory          │  │ Network   │ │
│  │ [=====>---] 45% │  │ [======>--] 60% │  │ ↓ 2.5MB/s│ │
│  └─────────────────┘  └─────────────────┘  │ ↑ 1.2MB/s│ │
│                                            └───────────┘ │
│                                                            │
│  ROS2 Nodes Status:                                       │
│  ┌────────────────────────────────────────────────────────┤
│  │ Node Name                    │ Status │ CPU  │ RAM    │ │
│  ├──────────────────────────────┼────────┼──────┼────────┤ │
│  │ /hardware_interface          │ 🟢 OK  │ 12%  │ 45MB   │ │
│  │ /mecanum_drive_controller    │ 🟢 OK  │ 8%   │ 32MB   │ │
│  │ /rplidar_node                │ 🟢 OK  │ 15%  │ 78MB   │ │
│  │ /cartographer_node           │ 🟢 OK  │ 25%  │ 210MB  │ │
│  │ /bt_navigator                │ 🟢 OK  │ 18%  │ 95MB   │ │
│  │ /waypoint_follower           │ 🟢 OK  │ 5%   │ 28MB   │ │
│  │ /qr_detection_node           │ 🟢 OK  │ 20%  │ 150MB  │ │
│  │ /actuator_control            │ 🟢 OK  │ 3%   │ 15MB   │ │
│  └──────────────────────────────┴────────┴──────┴────────┘ │
│                                                            │
│  Topics (Active):                                         │
│  /scan (10 Hz), /odom (50 Hz), /qr_detections (5 Hz)...  │
│                                                            │
│  [🔄 Refresh] [⚠️ View Errors] [📊 Detailed Stats]       │
└────────────────────────────────────────────────────────────┘
```

**Features:**
- Real-time system metrics
- Node health monitoring
- Topic frequency monitoring
- Error log viewer
- Resource usage graphs

---

## 🔧 Technical Implementation

### Backend API Endpoints

```python
# Flask/FastAPI Backend Structure

# Launch Management
POST   /api/launch/hardware          # Launch hardware.launch.py
POST   /api/launch/mapping           # Launch mapping mode
POST   /api/launch/navigation        # Launch navigation mode
POST   /api/launch/inventory         # Launch inventory mission
POST   /api/launch/stop/{process_id} # Stop a running launch
GET    /api/launch/status            # Get all launch statuses

# Navigation
POST   /api/nav/initial_pose         # Set initial pose
POST   /api/nav/goal                 # Send navigation goal
GET    /api/nav/pose                 # Get current robot pose
GET    /api/nav/path                 # Get current planned path

# Waypoints
GET    /api/waypoints                # Get all waypoints
POST   /api/waypoints                # Save waypoint
PUT    /api/waypoints/{id}           # Update waypoint
DELETE /api/waypoints/{id}           # Delete waypoint
POST   /api/waypoints/file           # Load from file
GET    /api/waypoints/file           # Download waypoints file

# QR Detection & Inventory
GET    /api/inventory/detections     # Get recent detections
GET    /api/inventory/stats          # Get statistics
POST   /api/inventory/export         # Export data
GET    /api/inventory/missions       # Get mission list
GET    /api/inventory/mission/{id}   # Get mission details

# Actuator Control
POST   /api/actuator/command         # Send actuator command (up/down/stop)
GET    /api/actuator/status          # Get actuator status

# System Monitoring
GET    /api/system/nodes             # List ROS2 nodes
GET    /api/system/topics            # List ROS2 topics
GET    /api/system/metrics           # System CPU/RAM/Network
GET    /api/system/logs              # Get recent logs

# WebSocket Endpoints
WS     /ws/camera                    # Live camera feed
WS     /ws/map                       # Live map updates
WS     /ws/detections                # Live QR detections
WS     /ws/logs                      # Live log streaming
WS     /ws/telemetry                 # Live robot telemetry
```

### Frontend Tech Stack

**Recommended: React + TypeScript**

```json
{
  "dependencies": {
    "react": "^18.2.0",
    "react-router-dom": "^6.20.0",
    "axios": "^1.6.0",
    "socket.io-client": "^4.6.0",
    "framer-motion": "^10.16.0",
    "@chakra-ui/react": "^2.8.0",
    "recharts": "^2.10.0",
    "react-leaflet": "^4.2.0",
    "react-dnd": "^16.0.1",
    "react-toastify": "^9.1.3"
  }
}
```

**Alternative: Vue 3 + TypeScript**

```json
{
  "dependencies": {
    "vue": "^3.3.0",
    "vue-router": "^4.2.0",
    "pinia": "^2.1.0",
    "axios": "^1.6.0",
    "socket.io-client": "^4.6.0",
    "chart.js": "^4.4.0",
    "vue-chartjs": "^5.2.0"
  }
}
```

---

## 🎨 Component Examples

### Launch Button Component (React)

```jsx
const LaunchButton = ({ launchType, params, onLaunch }) => {
  const [status, setStatus] = useState('idle'); // idle, launching, running, error
  const [output, setOutput] = useState([]);

  const handleLaunch = async () => {
    setStatus('launching');
    try {
      const response = await axios.post(`/api/launch/${launchType}`, params);
      setStatus('running');
      toast.success(`${launchType} launched successfully!`);
    } catch (error) {
      setStatus('error');
      toast.error(`Failed to launch: ${error.message}`);
    }
  };

  return (
    <div className="launch-card">
      <div className="card-header">
        <h3>{launchType.toUpperCase()}</h3>
        <StatusIndicator status={status} />
      </div>
      
      <div className="card-body">
        <ParamsEditor params={params} />
        <ConsoleOutput output={output} />
      </div>
      
      <div className="card-actions">
        <Button 
          variant="primary" 
          onClick={handleLaunch}
          disabled={status === 'launching'}
        >
          {status === 'launching' ? <Spinner /> : '▶ Launch'}
        </Button>
        {status === 'running' && (
          <Button variant="danger" onClick={() => stopLaunch()}>
            ⏹ Stop
          </Button>
        )}
      </div>
    </div>
  );
};
```

### Map Viewer Component (React + Leaflet)

```jsx
import { MapContainer, TileLayer, Marker, Polyline } from 'react-leaflet';

const MapViewer = ({ robotPose, waypoints, path }) => {
  const [clickMode, setClickMode] = useState('none'); // none, initial_pose, nav_goal, add_waypoint

  const handleMapClick = (e) => {
    const { lat, lng } = e.latlng;
    
    switch (clickMode) {
      case 'initial_pose':
        sendInitialPose(lat, lng);
        break;
      case 'nav_goal':
        sendNavGoal(lat, lng);
        break;
      case 'add_waypoint':
        addWaypoint(lat, lng);
        break;
    }
  };

  return (
    <div className="map-viewer">
      <div className="map-controls">
        <Button onClick={() => setClickMode('initial_pose')}>
          📍 Set Initial Pose
        </Button>
        <Button onClick={() => setClickMode('nav_goal')}>
          🎯 Send Goal
        </Button>
        <Button onClick={() => setClickMode('add_waypoint')}>
          ➕ Add Waypoint
        </Button>
      </div>
      
      <MapContainer center={[0, 0]} zoom={13}>
        <TileLayer url="..." />
        
        {/* Robot position */}
        <Marker position={[robotPose.x, robotPose.y]} icon={robotIcon}>
          <Popup>Robot</Popup>
        </Marker>
        
        {/* Waypoints */}
        {waypoints.map((wp, idx) => (
          <Marker key={idx} position={[wp.x, wp.y]} icon={waypointIcon}>
            <Popup>{wp.name}</Popup>
          </Marker>
        ))}
        
        {/* Planned path */}
        {path && <Polyline positions={path} color="#00d9ff" />}
      </MapContainer>
    </div>
  );
};
```

### Real-time Detection Feed (React)

```jsx
const DetectionFeed = () => {
  const [detections, setDetections] = useState([]);
  
  useEffect(() => {
    const socket = io('/ws/detections');
    
    socket.on('qr_detected', (data) => {
      setDetections(prev => [data, ...prev].slice(0, 50)); // Keep last 50
      
      // Show notification
      toast.info(
        `QR Detected: ${data.rack_id}-${data.shelf_id}-${data.item_code}`,
        { icon: '📦' }
      );
      
      // Play sound
      playDetectionSound();
    });
    
    return () => socket.disconnect();
  }, []);

  return (
    <div className="detection-feed">
      <h3>Live Detections</h3>
      <div className="feed-list">
        {detections.map((det, idx) => (
          <motion.div 
            key={idx}
            className="detection-item"
            initial={{ x: -100, opacity: 0 }}
            animate={{ x: 0, opacity: 1 }}
            transition={{ duration: 0.3 }}
          >
            <span className="time">{det.timestamp}</span>
            <span className="rack">{det.rack_id}</span>
            <span className="item">{det.item_code}</span>
            <span className="confidence">
              {(det.confidence * 100).toFixed(0)}%
            </span>
          </motion.div>
        ))}
      </div>
    </div>
  );
};
```

---

## 🎨 CSS Styling (Cyberpunk Theme)

```css
/* Global Styles */
:root {
  --bg-primary: #0a0e27;
  --bg-secondary: #1a1f3a;
  --bg-card: #252a4a;
  --accent-cyan: #00d9ff;
  --accent-purple: #bd00ff;
  --accent-green: #00ff88;
  --accent-orange: #ffaa00;
  --accent-red: #ff0055;
  --text-primary: #ffffff;
  --text-secondary: #a0a6cc;
  --border-color: rgba(0, 217, 255, 0.2);
  --shadow-glow: 0 0 20px rgba(0, 217, 255, 0.3);
}

body {
  background: var(--bg-primary);
  color: var(--text-primary);
  font-family: 'Rajdhani', sans-serif;
  overflow-x: hidden;
}

/* Animated background grid */
.bg-grid {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-image: 
    linear-gradient(var(--accent-cyan) 1px, transparent 1px),
    linear-gradient(90deg, var(--accent-cyan) 1px, transparent 1px);
  background-size: 50px 50px;
  opacity: 0.05;
  animation: gridMove 20s linear infinite;
}

@keyframes gridMove {
  0% { transform: translate(0, 0); }
  100% { transform: translate(50px, 50px); }
}

/* Glass card effect */
.card {
  background: rgba(37, 42, 74, 0.6);
  backdrop-filter: blur(10px);
  border: 1px solid var(--border-color);
  border-radius: 16px;
  padding: 24px;
  box-shadow: var(--shadow-glow);
  transition: all 0.3s ease;
}

.card:hover {
  transform: translateY(-4px);
  box-shadow: 0 0 30px rgba(0, 217, 255, 0.5);
  border-color: var(--accent-cyan);
}

/* Neon button */
.btn-primary {
  background: linear-gradient(135deg, var(--accent-cyan), var(--accent-purple));
  border: none;
  color: white;
  padding: 12px 32px;
  border-radius: 8px;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 1px;
  cursor: pointer;
  position: relative;
  overflow: hidden;
  transition: all 0.3s ease;
}

.btn-primary::before {
  content: '';
  position: absolute;
  top: 0;
  left: -100%;
  width: 100%;
  height: 100%;
  background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
  transition: left 0.5s;
}

.btn-primary:hover::before {
  left: 100%;
}

.btn-primary:hover {
  box-shadow: 0 0 20px var(--accent-cyan);
  transform: scale(1.05);
}

/* Status indicator with pulse animation */
.status-indicator {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  display: inline-block;
  margin-right: 8px;
}

.status-indicator.active {
  background: var(--accent-green);
  box-shadow: 0 0 10px var(--accent-green);
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

/* Holographic text effect */
.holo-text {
  background: linear-gradient(45deg, var(--accent-cyan), var(--accent-purple), var(--accent-green));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  font-weight: 700;
  font-size: 2.5rem;
  text-shadow: 0 0 20px rgba(0, 217, 255, 0.5);
}

/* Console output */
.console-output {
  background: rgba(0, 0, 0, 0.5);
  border: 1px solid var(--accent-cyan);
  border-radius: 8px;
  padding: 16px;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  max-height: 300px;
  overflow-y: auto;
  color: var(--accent-green);
}

.console-output::-webkit-scrollbar {
  width: 8px;
}

.console-output::-webkit-scrollbar-thumb {
  background: var(--accent-cyan);
  border-radius: 4px;
}

/* Animated loading spinner */
.spinner {
  border: 3px solid rgba(0, 217, 255, 0.1);
  border-top: 3px solid var(--accent-cyan);
  border-radius: 50%;
  width: 40px;
  height: 40px;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

/* Glitch effect for headers */
.glitch {
  position: relative;
  animation: glitch 3s infinite;
}

@keyframes glitch {
  0%, 90%, 100% {
    transform: translate(0);
    text-shadow: none;
  }
  92% {
    transform: translate(-2px, 2px);
    text-shadow: 2px -2px var(--accent-cyan), -2px 2px var(--accent-purple);
  }
  94% {
    transform: translate(2px, -2px);
    text-shadow: -2px 2px var(--accent-cyan), 2px -2px var(--accent-purple);
  }
}
```

---

## 📦 File Structure

```
warehouse-rover-gui/
├── backend/
│   ├── app.py                    # Flask/FastAPI main app
│   ├── ros2_bridge.py            # ROS2 communication layer
│   ├── launch_manager.py         # Launch file management
│   ├── database.py               # Database interface
│   ├── websocket_handler.py      # WebSocket connections
│   └── requirements.txt
│
├── frontend/
│   ├── public/
│   │   ├── index.html
│   │   ├── favicon.ico
│   │   └── assets/
│   │       ├── robot-icon.svg
│   │       └── sounds/
│   │           ├── detection.mp3
│   │           └── notification.mp3
│   │
│   ├── src/
│   │   ├── components/
│   │   │   ├── Dashboard/
│   │   │   │   ├── Dashboard.jsx
│   │   │   │   ├── SystemStats.jsx
│   │   │   │   ├── QuickLaunch.jsx
│   │   │   │   └── ActivityLog.jsx
│   │   │   │
│   │   │   ├── LaunchControl/
│   │   │   │   ├── LaunchControl.jsx
│   │   │   │   ├── LaunchCard.jsx
│   │   │   │   ├── ConsoleOutput.jsx
│   │   │   │   └── ParameterEditor.jsx
│   │   │   │
│   │   │   ├── Navigation/
│   │   │   │   ├── NavigationControl.jsx
│   │   │   │   ├── MapViewer.jsx
│   │   │   │   ├── TeleopControl.jsx
│   │   │   │   └── WaypointList.jsx
│   │   │   │
│   │   │   ├── WaypointCreator/
│   │   │   │   ├── WaypointCreator.jsx
│   │   │   │   ├── InteractiveMap.jsx
│   │   │   │   └── WaypointEditor.jsx
│   │   │   │
│   │   │   ├── Inventory/
│   │   │   │   ├── InventoryDashboard.jsx
│   │   │   │   ├── CameraFeed.jsx
│   │   │   │   ├── DetectionFeed.jsx
│   │   │   │   └── DatabaseViewer.jsx
│   │   │   │
│   │   │   ├── System/
│   │   │   │   ├── SystemMonitor.jsx
│   │   │   │   ├── NodeStatus.jsx
│   │   │   │   └── MetricsChart.jsx
│   │   │   │
│   │   │   └── Common/
│   │   │       ├── Header.jsx
│   │   │       ├── Sidebar.jsx
│   │   │       ├── Button.jsx
│   │   │       ├── StatusIndicator.jsx
│   │   │       └── LoadingSpinner.jsx
│   │   │
│   │   ├── hooks/
│   │   │   ├── useROS2.js
│   │   │   ├── useWebSocket.js
│   │   │   └── useLaunchManager.js
│   │   │
│   │   ├── services/
│   │   │   ├── api.js
│   │   │   ├── websocket.js
│   │   │   └── storage.js
│   │   │
│   │   ├── styles/
│   │   │   ├── global.css
│   │   │   ├── theme.css
│   │   │   └── animations.css
│   │   │
│   │   ├── utils/
│   │   │   ├── formatters.js
│   │   │   └── validators.js
│   │   │
│   │   ├── App.jsx
│   │   └── main.jsx
│   │
│   ├── package.json
│   └── vite.config.js
│
├── docker-compose.yml
├── Dockerfile
└── README.md
```

---

## 🚀 Getting Started Instructions

```markdown
# Warehouse Rover Control GUI

## Prerequisites
- Python 3.10+
- Node.js 18+
- ROS2 Humble
- Docker (optional)

## Installation

### Backend Setup
```bash
cd backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Frontend Setup
```bash
cd frontend
npm install
```

## Running

### Development Mode
```bash
# Terminal 1: Backend
cd backend
source venv/bin/activate
python app.py

# Terminal 2: Frontend
cd frontend
npm run dev
```

### Production Mode
```bash
# Build frontend
cd frontend
npm run build

# Run with Docker
docker-compose up
```

## Access
- Frontend: http://localhost:3000
- Backend API: http://localhost:5000
- WebSocket: ws://localhost:5000

## Configuration
Edit `config.yaml` to customize:
- ROS2 workspace path
- Launch file locations
- Database connection
- WebSocket ports
```

---

## 🎯 Key Features Summary

1. ✅ **One-click launch** of all ROS2 systems
2. ✅ **Real-time monitoring** of all nodes and topics
3. ✅ **Interactive map** with click-to-navigate
4. ✅ **Waypoint creator** with teleop control
5. ✅ **Live camera feed** with QR detection overlay
6. ✅ **Database viewer** with search and export
7. ✅ **System metrics** dashboard
8. ✅ **Dark cyberpunk theme** with animations
9. ✅ **Responsive design** for all devices
10. ✅ **WebSocket streaming** for real-time data

---

## 🎨 Additional Polish

### Sound Effects
- Detection sound when QR code found
- Success chime when mission complete
- Alert sound for errors
- Ambient background music (optional)

### Animations
- Smooth page transitions
- Loading states with spinners
- Toast notifications
- Progress bars
- Robot movement visualization

### Easter Eggs
- Konami code for hidden features
- Robot ASCII art in console
- Matrix rain effect option
- Voice control (optional)

---

## 📚 References & Inspiration

- [Team Deimos Robotic Arm GUI](https://github.com/Team-Deimos-IIT-Mandi/Robotic-Arm-Gui)
- [Nav2 Web UI](https://github.com/ros-planning/navigation2/tree/main/nav2_simple_commander)
- [ROS2 Web Bridge](https://github.com/RobotWebTools/rosbridge_suite)
- [Cyberpunk 2077 UI/UX](https://www.behance.net/search/projects?search=cyberpunk+ui)

---

## 🎓 Development Tips

1. Start with **Dashboard** and **Launch Control** pages first
2. Use **dummy data** initially, connect to ROS2 later
3. Implement **WebSocket** for real-time updates
4. Add **error handling** for all API calls
5. Use **TypeScript** for better code quality
6. Add **unit tests** for critical components
7. Optimize **performance** for smooth animations
8. Test on **different screen sizes**

---

## 🚀 Deployment

### Option 1: Local Development
```bash
npm run dev
python app.py
```

### Option 2: Docker
```bash
docker-compose up --build
```

### Option 3: Production Server
```bash
npm run build
gunicorn -w 4 -k uvicorn.workers.UvicornWorker app:app
```

---

## 🎉 Final Result

You'll have a **sleek, modern, cyberpunk-themed GUI** that:
- Makes controlling the warehouse rover **intuitive and fun**
- Provides **real-time feedback** on all operations
- Looks **professional and futuristic**
- Is **easy to use** for operators
- Has **all features accessible** via beautiful UI

**Good luck building! 🤖✨**
