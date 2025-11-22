# Navigation Setup Package

ROS2 navigation configuration for the mecanum robot using Cartographer SLAM and Nav2 autonomous navigation.

## Overview

This package provides complete navigation stack configuration including:
- **Cartographer**: SLAM and localization with sensor fusion
- **Nav2**: Path planning, obstacle avoidance, and behavior trees
- **AMCL**: Monte Carlo localization for saved maps
- **Map Server**: Serves pre-built maps for navigation

## Package Structure

```
navigation_setup/
├── config/
│   ├── cartographer_mapping.lua      # SLAM mapping config
│   ├── cartographer_odom.lua         # Odometry fusion config
│   ├── nav2_params.yaml              # Nav2 stack parameters
│   └── amcl_params.yaml              # AMCL localization (optional)
├── launch/
│   ├── cartographer_mapping.launch.py   # Mapping mode
│   ├── cartographer_navigation.launch.py # Navigation with AMCL
│   └── nav2_bringup.launch.py          # Nav2 stack only
├── maps/
│   ├── my_map.pgm                    # Saved map image
│   └── my_map.yaml                   # Map metadata
├── rviz/
│   ├── cartographer.rviz             # RViz config for mapping
│   └── navigation.rviz               # RViz config for navigation
└── CMakeLists.txt
```

## Cartographer Configuration

### Two-Config Strategy

This project uses **two separate Cartographer configurations** to avoid TF conflicts:

| Mode | Config File | TF Published | Use Case |
|------|-------------|--------------|----------|
| **Mapping** | `cartographer_mapping.lua` | `map → odom → base_link` | Building new maps |
| **Navigation** | `cartographer_odom.lua` | `odom → base_link` only | Autonomous driving |

**Why?** 
- In navigation mode, both AMCL and Cartographer would try to publish `map → odom`
- Solution: Cartographer only does sensor fusion (`odom → base_link`), AMCL does localization (`map → odom`)

**See**: [CARTOGRAPHER_CONFIGS.md](../../CARTOGRAPHER_CONFIGS.md) for detailed explanation

### 1. cartographer_mapping.lua

**Purpose**: Full SLAM for building maps

**Key Settings**:
```lua
options = {
  map_frame = "map",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,  -- Publishes full TF chain
  use_odometry = true,         -- Fuse wheel odometry
  
  num_laser_scans = 1,
  num_point_clouds = 0,
}

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4  # Use 2 for RPi5
```

**TF Chain**:
```
map → odom → base_link → [sensors, wheels]
└──── All published by Cartographer ────┘
```

**Topics**:
- Input: `/scan`, `/odom`, `/imu/data`
- Output: `/map` (occupancy grid), `/submap_list`, `/landmark_poses_list`
- TF: `map → odom → base_link`

**Launch**:
```bash
# Simulation
ros2 launch navigation_setup cartographer_mapping.launch.py use_sim:=true

# Hardware
ros2 launch mecanum_hardware hardware_mapping.launch.py
```

### 2. cartographer_odom.lua

**Purpose**: Sensor fusion for navigation (no mapping)

**Key Settings**:
```lua
options = {
  map_frame = "odom",           -- Cartographer's "map" is odom frame
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,    -- Publishes odom → base_link only
  use_odometry = true,
}

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
```

**TF Chain**:
```
map → odom → base_link
(AMCL) (Cartographer)
```

**Topics**:
- Input: `/scan`, `/odom`, `/imu/data`
- Output: TF only (`odom → base_link`)

**Launch**:
```bash
# Always launched as part of navigation stack
ros2 launch mecanum_hardware hardware_navigation.launch.py map:=/path/to/map.yaml
```

## Nav2 Configuration

### File: `config/nav2_params.yaml`

Complete Nav2 stack configuration with:
- Global costmap
- Local costmap
- Path planners
- Controllers
- Behavior trees
- Recovery behaviors

**Key Parameters**:

**Global Costmap** (for path planning):
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      width: 10
      height: 10
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

**Local Costmap** (for obstacle avoidance):
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      resolution: 0.05
      width: 3
      height: 3
      plugins: ["voxel_layer", "inflation_layer"]
```

**Controller** (trajectory following):
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      max_vel_y: 0.5
      max_vel_theta: 1.0
      min_vel_x: -0.5
      min_vel_y: -0.5
      min_vel_theta: -1.0
```

**Planner** (global path):
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false  # Dijkstra
```

**Behavior Tree Navigator**:
```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /mecanum_drive_controller/odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
```

## Mapping Workflow

### 1. Launch Mapping Mode

**Simulation**:
```bash
# Terminal 1: Gazebo
ros2 launch mecanum_in_gazebo gazebo.launch.py

# Terminal 2: Cartographer
ros2 launch navigation_setup cartographer_mapping.launch.py use_sim:=true

# Terminal 3: Teleop
ros2 run mecanum_in_gazebo mecanum_teleop_key.py
```

**Hardware**:
```bash
# On Raspberry Pi 5 (in Docker)
ros2 launch mecanum_hardware hardware_mapping.launch.py

# On laptop (same network)
export ROS_DOMAIN_ID=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard
rviz2  # Load cartographer.rviz config
```

### 2. Drive and Build Map

- Drive robot around the environment
- Cover all areas you want to map
- Drive slowly for better accuracy
- Close loops (return to starting point) for optimization
- Monitor map quality in RViz

**RViz Setup**:
- Fixed Frame: `map`
- Add Display: `Map` topic `/map`
- Add Display: `LaserScan` topic `/scan`
- Add Display: `RobotModel`
- Add Display: `TF`

### 3. Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

**Output Files**:
- `my_map.pgm` - Map image (grayscale)
- `my_map.yaml` - Map metadata

**Map YAML Example**:
```yaml
image: my_map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### 4. Copy Map to Package

```bash
# On hardware
cp ~/my_map.* /workspace/src/navigation_setup/maps/

# On simulation PC
cp ~/my_map.* ~/002/src/navigation_setup/maps/
```

## Navigation Workflow

### 1. Launch Navigation Mode

**Simulation**:
```bash
# Terminal 1: Gazebo
ros2 launch mecanum_in_gazebo gazebo.launch.py

# Terminal 2: Navigation stack
ros2 launch navigation_setup cartographer_navigation.launch.py \
  use_sim:=true \
  map:=~/002/src/navigation_setup/maps/my_map.yaml
```

**Hardware**:
```bash
# On Raspberry Pi 5
ros2 launch mecanum_hardware hardware_navigation.launch.py \
  map:=/workspace/src/navigation_setup/maps/my_map.yaml
```

### 2. Set Initial Pose (RViz)

1. Open RViz on laptop
2. Click **"2D Pose Estimate"** button (top toolbar)
3. Click on map where robot is located
4. Drag to set orientation
5. AMCL particle cloud should converge around robot

### 3. Send Navigation Goal

**Method 1: RViz**
1. Click **"Nav2 Goal"** button
2. Click destination on map
3. Drag to set goal orientation
4. Robot plans path and navigates autonomously

**Method 2: Command Line**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, \
   pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
   orientation: {w: 1.0}}}}"
```

**Method 3: Python Script**
```python
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

rclpy.init()
navigator = BasicNavigator()

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 1.0
goal_pose.pose.orientation.w = 1.0

navigator.goToPose(goal_pose)

while not navigator.isTaskComplete():
    rclpy.spin_once(navigator)

print(navigator.getResult())
```

## Launch Files

### cartographer_mapping.launch.py

**Purpose**: Launch Cartographer for SLAM mapping

**Arguments**:
- `use_sim` (bool): true for simulation, false for hardware
- `resolution` (float): Submap resolution (default: 0.05)
- `publish_period_sec` (float): Submap publish period (default: 1.0)

**Nodes**:
1. `cartographer_node` - SLAM algorithm
2. `cartographer_occupancy_grid_node` - Generates occupancy grid

**Usage**:
```bash
ros2 launch navigation_setup cartographer_mapping.launch.py \
  use_sim:=true \
  resolution:=0.05
```

### cartographer_navigation.launch.py

**Purpose**: Launch navigation with AMCL + Cartographer sensor fusion

**Arguments**:
- `use_sim` (bool): Simulation or hardware
- `map` (string): Path to map YAML file
- `params_file` (string): Nav2 parameters (default: nav2_params.yaml)
- `autostart` (bool): Auto-start Nav2 nodes (default: true)

**Nodes**:
1. `cartographer_node` - Sensor fusion only
2. `map_server` - Serves saved map
3. `amcl` - Monte Carlo localization
4. `controller_server` - Path following
5. `planner_server` - Path planning
6. `recoveries_server` - Recovery behaviors
7. `bt_navigator` - Behavior tree navigation
8. `waypoint_follower` - Waypoint navigation
9. `lifecycle_manager` - Manages node lifecycle

**Usage**:
```bash
ros2 launch navigation_setup cartographer_navigation.launch.py \
  use_sim:=true \
  map:=/path/to/my_map.yaml
```

### nav2_bringup.launch.py

**Purpose**: Launch Nav2 stack only (no Cartographer)

Use when you already have odometry and want just Nav2.

**Usage**:
```bash
ros2 launch navigation_setup nav2_bringup.launch.py \
  map:=/path/to/my_map.yaml \
  params_file:=/path/to/nav2_params.yaml
```

## Topics Reference

### Cartographer Topics

**Subscribed**:
- `/scan` (sensor_msgs/LaserScan) - LiDAR data
- `/odom` (nav_msgs/Odometry) - Wheel odometry
- `/imu/data` (sensor_msgs/Imu) - IMU data

**Published**:
- `/map` (nav_msgs/OccupancyGrid) - SLAM map
- `/submap_list` (cartographer_ros_msgs/SubmapList)
- `/landmark_poses_list` (cartographer_ros_msgs/LandmarkList)
- `/constraint_list` (visualization_msgs/MarkerArray)
- `/trajectory_node_list` (visualization_msgs/MarkerArray)

**Services**:
- `/submap_query` - Query submap
- `/finish_trajectory` - Finish current trajectory
- `/write_state` - Save SLAM state

### Nav2 Topics

**Subscribed**:
- `/map` - Static map from map_server
- `/scan` - Obstacle detection
- `/odom` - Robot odometry

**Published**:
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/global_costmap/costmap` - Global planning costmap
- `/local_costmap/costmap` - Local obstacle costmap
- `/plan` (nav_msgs/Path) - Planned path
- `/local_plan` (nav_msgs/Path) - Local trajectory

**Actions**:
- `/navigate_to_pose` - Navigate to goal pose
- `/follow_waypoints` - Follow waypoint list
- `/compute_path_to_pose` - Plan path only (no execution)

### AMCL Topics

**Subscribed**:
- `/scan` - For localization
- `/map` - Static map
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - Initial pose

**Published**:
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) - Robot pose
- `/particlecloud` (geometry_msgs/PoseArray) - Particle filter cloud

## TF Trees

### Mapping Mode

```
map
 └─ odom (Cartographer)
     └─ base_link (Cartographer)
         ├─ imu_link
         ├─ lidar_link
         ├─ camera_link
         └─ wheel_frames
```

**All TF published by Cartographer**

### Navigation Mode

```
map
 ├─ odom (AMCL)
 │   └─ base_link (Cartographer)
 │       ├─ imu_link
 │       ├─ lidar_link
 │       ├─ camera_link
 │       └─ wheel_frames
 └─ (no direct connection)
```

**TF Publishers**:
- `map → odom`: AMCL (localization on saved map)
- `odom → base_link`: Cartographer (sensor fusion)
- `base_link → sensors`: robot_state_publisher (from URDF)

## Monitoring & Debugging

### Check Navigation Status

```bash
# List Nav2 nodes
ros2 node list | grep -E "(nav2|amcl|map_server|cartographer)"

# Check lifecycle states
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
ros2 lifecycle list /amcl

# Monitor navigation action
ros2 action list
ros2 action info /navigate_to_pose

# Check costmaps
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once

# Monitor AMCL pose
ros2 topic echo /amcl_pose
```

### Check Cartographer

```bash
# Check Cartographer node
ros2 node info /cartographer_node

# Monitor map
ros2 topic hz /map
ros2 topic echo /map --once

# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Verify TF publishers
ros2 run tf2_ros tf2_echo map odom      # Should show AMCL
ros2 run tf2_ros tf2_echo odom base_link # Should show Cartographer
```

### Visualize in RViz

**Mapping Config** (`rviz/cartographer.rviz`):
- Fixed Frame: `map`
- Displays: Map, LaserScan, RobotModel, TF, Odometry

**Navigation Config** (`rviz/navigation.rviz`):
- Fixed Frame: `map`
- Displays: Map, LaserScan, RobotModel, TF, Path, ParticleCloud, Costmaps

## Troubleshooting

### Problem: TF_REPEATED_DATA error
**Cause**: Both Cartographer and AMCL publishing `map → odom`  
**Solution**: Use `cartographer_odom.lua` in navigation mode

### Problem: AMCL doesn't converge
**Solutions**:
1. Set better initial pose in RViz
2. Increase AMCL particle count in `amcl_params.yaml`
3. Drive robot around to get more scan matches

### Problem: Robot gets stuck
**Solutions**:
1. Tune DWB controller parameters (velocity limits)
2. Adjust inflation radius in costmap
3. Enable recovery behaviors

### Problem: Poor map quality
**Solutions**:
1. Drive slower during mapping
2. Increase scan matching resolution
3. Close loops (return to starting point)
4. Reduce `imu_gravity_time_constant` for better IMU fusion

### Problem: Path planning fails
**Solutions**:
1. Check global costmap for obstacles
2. Increase planning tolerance
3. Use A* instead of Dijkstra: `use_astar: true`

## Performance Tuning

### For Raspberry Pi 5

**Cartographer**:
```lua
MAP_BUILDER.num_background_threads = 2  # Reduce from 4
POSE_GRAPH.optimize_every_n_nodes = 120  # Increase from 90
```

**Nav2**:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Reduce from 20.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0  # Reduce from 20.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5  # Reduce from 1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 2.0  # Reduce from 5.0
```

## Related Documentation

- [CARTOGRAPHER_CONFIGS.md](../../CARTOGRAPHER_CONFIGS.md) - Cartographer strategy explained
- [mecanum_hardware/README.md](../mecanum_hardware/README.md) - Hardware interface
- [mecanum_in_gazebo/README.md](../mecanum_in_gazebo/README.md) - Simulation
- [ARCHITECTURE.md](../../ARCHITECTURE.md) - System architecture

## Dependencies

**ROS2 Packages**:
- `cartographer_ros`
- `nav2_bringup`
- `nav2_bt_navigator`
- `nav2_controller`
- `nav2_planner`
- `nav2_recoveries`
- `nav2_map_server`
- `nav2_lifecycle_manager`
- `nav2_amcl`
- `nav2_simple_commander` (for Python API)

**System**:
- Adequate CPU for SLAM (RPi5: 2 background threads)
- Network connectivity for wireless RViz

---

**Author**: Mecanum Robot Team  
**Last Updated**: November 2025  
**ROS2 Version**: Humble Hawksbill
