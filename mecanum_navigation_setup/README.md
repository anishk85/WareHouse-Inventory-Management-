# Mecanum Navigation Setup

ROS 2 navigation configuration package for the mecanum-wheeled robot with SLAM and autonomous navigation capabilities.

## Overview

This package provides the navigation stack for both mapping and autonomous navigation:
- **Cartographer**: Real-time SLAM for environment mapping
- **Nav2**: Path planning, obstacle avoidance, and autonomous navigation
- **AMCL**: Localization on pre-built maps

## Package Contents

```
mecanum_navigation_setup/
├── config/
│   ├── cartographer*.lua         # Cartographer SLAM configurations
│   ├── mapper_params*.yaml       # SLAM Toolbox parameters
│   ├── nav2_params.yaml          # Nav2 navigation parameters
│   └── nav2_params_stvl.yaml     # Alternative Nav2 config
├── launch/
│   ├── slam.launch.py            # SLAM Toolbox mapping
│   ├── cartographer.launch.py   # Cartographer SLAM
│   ├── navigation.launch.py     # Nav2 with AMCL
│   ├── navigation_stvl.launch.py # Alternative navigation
│   └── amcl_navigation.launch.py # AMCL localization
├── maps/                         # Saved maps (PGM + YAML)
├── rviz/                         # RViz configurations
└── scripts/                      # Helper scripts
```

## Quick Start

### 1. Mapping (Create a New Map)

**Simulation:**
```bash
# Terminal 1: Start simulation
ros2 launch mecanum_gazebo simulation_world.launch.py

# Terminal 2: Start SLAM
ros2 launch mecanum_navigation_setup navigation.launch.py use_sim_time:=true

# Terminal 3: Teleoperate robot to explore environment
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Save the map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_warehouse_map
# Copy map files to mecanum_navigation_setup/maps/
```

### 2. Navigation (Autonomous Navigation on Saved Map)

**Simulation:**
```bash
# Terminal 1: Start simulation
ros2 launch mecanum_gazebo simulation_world.launch.py

# Terminal 2: Start navigation
ros2 launch mecanum_navigation_setup cartographer_navigation.launch.py \
  use_sim_time:=true \
  map:=<path_to_map.yaml>
```

**Hardware:**
```bash
# On robot
ros2 launch mecanum_bringup hardware.launch.py

# On laptop
ros2 launch mecanum_navigation_setup navigation.launch.py \
  use_sim_time:=false \
  map:=<path_to_map.yaml> \
  use_rviz:=true
```

**Send navigation goals in RViz:**
1. Click "2D Pose Estimate" to set initial robot position
2. Click "Nav2 Goal" to send the robot to a destination

## Configuration Files

### SLAM Configurations

- **`cartographer.lua`** / **`cartographer_hardware.lua`**: Cartographer SLAM for real-time mapping
- **`cartographer_mapping.lua`**: Full SLAM with map→odom→base_link TF chain
- **`cartographer_odom.lua`**: Sensor fusion only (odom→base_link) for use with AMCL

### Navigation Configurations

- **`nav2_params.yaml`**: Complete Nav2 stack configuration
  - Global/local costmaps with obstacle layers
  - DWB controller for holonomic motion
  - NavFn planner for path planning
  - Recovery behaviors and behavior trees
  
- **`nav2_params_stvl.yaml`**: Alternative Nav2 configuration (STVL costmap)

**Key Nav2 Components:**
- **Controller**: Holonomic velocity control for mecanum drive
- **Planner**: Global path planning (Dijkstra/A*)
- **Costmaps**: Obstacle detection and inflation
- **AMCL**: Particle filter localization on static maps
- **Behavior Trees**: High-level navigation logic

## Launch Files Reference

| Launch File | Purpose | Key Arguments |
|-------------|---------|---------------|
| `cartographer.launch.py` | Cartographer SLAM | `use_sim_time`, `resolution` |
| `cartographer_navigation.launch.py` | Cartographer + AMCL + Nav2 | `use_sim`, `map`, `params_file` |

### Common Launch Arguments

- **`use_sim_time`** (bool): Use simulation clock (default: true)
- **`map`** (string): Path to map YAML file
- **`params_file`** (string): Path to Nav2 parameters file
- **`use_rviz`** (bool): Launch RViz visualization (default: true)
- **`autostart`** (bool): Auto-start lifecycle nodes (default: true)

## Key Topics and Services

### Important Python Files

```bash
python ~/inter_iit_ws/src/mecanum_navigation_setup/maps/updated_map_for_tape.py
```

This creates the new_map.pgm which has bounds for tape
Remember to update the map.yaml inside 'cartographer_navigation.launch.py if not using tape bounds

### Important Topics

**Sensor Inputs:**
- `/scan` - LiDAR laser scan data
- `/odom` - Wheel odometry
- `/imu/data` - IMU orientation and angular velocity

**Navigation:**
- `/cmd_vel` - Velocity commands to robot
- `/map` - Occupancy grid map
- `/plan` - Global path plan
- `/local_plan` - Local trajectory

**Localization:**
- `/amcl_pose` - Robot pose estimate from AMCL
- `/particlecloud` - AMCL particle filter visualization

### Navigation Actions

- `/navigate_to_pose` - Send robot to a goal pose
- `/follow_waypoints` - Navigate through multiple waypoints
- `/compute_path_to_pose` - Plan path without execution

## TF Frame Structure

**Mapping Mode:**
```
map → odom → base_link → [sensor frames]
      └─ Published by SLAM
```

**Navigation Mode:**
```
map → odom → base_link → [sensor frames]
 (AMCL)  (Sensor fusion/odometry)
```
