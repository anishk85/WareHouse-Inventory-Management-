# Mecanum Description Package

## Overview

The `mecanum_description` package provides the complete robot description for a mecanum-wheeled mobile manipulator designed for autonomous warehouse operations. This package contains URDF/Xacro models, sensor configurations, controller setups, RViz visualizations, and essential bridge nodes for both simulation and hardware deployment.

**Key Features:**
- Complete URDF/Xacro robot model with mecanum drive kinematics
- Integrated sensor suite (LiDAR, depth camera, IMU, lift camera)
- ROS 2 Control integration for both simulation and hardware
- Configurable twist multiplexer for teleoperation priority
- Utility nodes for odometry publishing, sensor bridging, and waypoint recording
- Pre-configured RViz setups for SLAM, navigation, and visualization

---

## Table of Contents

- [Package Structure](#package-structure)
- [Robot Description](#robot-description)
- [Sensors](#sensors)
- [Controllers](#controllers)
- [Important Scripts and Nodes](#Important-Scripts-and-Nodes)
- [Configuration Files](#configuration-files)
- [RViz Configurations](#rviz-configurations)

---

## Package Structure

```
mecanum_description/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package manifest
├── readme.md                   # This file
├── config/                     # Configuration files
│   ├── controllers.yaml        # ROS 2 Control configuration
│   └── twist_mux.yaml          # Twist multiplexer configuration
├── meshes/                     # 3D mesh files for visualization
├── rviz/                       # RViz configuration files
│   ├── cartographer.rviz       # Cartographer SLAM visualization
│   ├── nav2_stvl.rviz          # Nav2 with Space-Time Voxel Layer
│   ├── nav2.rviz               # Standard Nav2 visualization
│   └── slam.rviz               # Generic SLAM visualization
├── scripts/                    # Python utility nodes
│   ├── cmd_vel_bridge.py       # Command velocity bridge
│   ├── imu_covariance_fixer.py # IMU data covariance correction
│   ├── move_qr_camera.py       # QR camera positioning control
│   ├── odom_tf_publisher.py    # Odometry TF broadcaster
│   ├── rack_waypoints.py       # Waypoint recording utility
│   ├── return_to_spawn.py      # Return robot to spawn position
│   └── sensor_bridge.py        # Sensor data bridging node
└── urdf/                       # Robot description files
    ├── depth_camera.gazebo     # Depth camera Gazebo plugin
    ├── depth_camera.xacro      # Depth camera URDF description
    ├── imu.gazebo              # IMU Gazebo plugin
    ├── imu.xacro               # IMU URDF description
    ├── lidar.gazebo            # LiDAR Gazebo plugin
    ├── lidar.xacro             # LiDAR URDF description
    ├── lift_camera.gazebo      # Lift camera Gazebo plugin
    ├── lift_camera.xacro       # Lift camera URDF description
    ├── lift.xacro              # Lift mechanism description
    ├── materials.xacro         # Material definitions
    ├── mec_rob.gazebo          # Main robot Gazebo configuration
    ├── mec_rob.ros2_control.hardware.xacro  # Hardware interface
    ├── mec_rob.ros2_control.xacro           # Simulation interface
    ├── mec_rob.trans           # Transmission definitions
    └── mec_rob.xacro           # Main robot URDF file
```

---

## Robot Description

### Main Robot Model (`mec_rob.xacro`)

The robot model is a mecanum-wheeled mobile platform with:
- **Drive System**: 4-wheel mecanum drive for omnidirectional movement
- **Lift Mechanism**: Vertical actuator for multi-height scanning
- **Modular Design**: Xacro-based with conditional hardware/simulation loading

### Key Features:

1. **Conditional Hardware Loading**:
   ```xml
   <xacro:arg name="use_sim" default="true"/>
   ```
   - `use_sim=true`: Loads simulation controllers (Gazebo)
   - `use_sim=false`: Loads hardware interface (ESP32 stepper controllers)

2. **Mecanum Wheel Kinematics**:
   - Front Left, Front Right, Rear Left, Rear Right wheels
   - Configured for omnidirectional movement
   - Wheel diameter and separation defined in robot parameters

3. **Lift Mechanism**:
   - Prismatic joint for vertical movement
   - Controlled via `lift_position_controller`
   - Enables multi-level rack scanning

---

## Sensors

### 1. **LiDAR** (`lidar.xacro`, `lidar.gazebo`)
- **Type**: 2D scanning laser rangefinder
- **Topic**: `/scan`
- **Use Case**: Navigation, obstacle avoidance, SLAM

### 2. **Depth Camera** (`depth_camera.xacro`, `depth_camera.gazebo`)
- **Type**: RGB-D camera
- **Topics**: 
  - `/camera/image_raw`
  - `/camera/depth/image_raw`
  - `/camera/points`
- **Use Case**: Front-facing perception, obstacle detection

### 3. **Lift Camera** (`lift_camera.xacro`, `lift_camera.gazebo`)
- **Type**: RGB camera on lift mechanism
- **Topic**: `/lift_camera/image_raw`
- **Use Case**: QR code scanning at different heights

### 4. **IMU** (`imu.xacro`, `imu.gazebo`)
- **Type**: Inertial Measurement Unit
- **Topic**: `/imu/data`
- **Use Case**: Orientation estimation, sensor fusion

---

## Controllers

### ROS 2 Control Configuration (`config/controllers.yaml`)

#### 1. **Joint State Broadcaster**
- **Type**: `joint_state_broadcaster/JointStateBroadcaster`
- **Rate**: 100 Hz
- **Function**: Publishes joint states for visualization

#### 2. **Mecanum Drive Controller**
- **Type**: `mecanum_drive_controller/MecanumDriveController`
- **Input Topic**: `/cmd_vel`
- **Controlled Joints**:
  - `front_left_joint`
  - `front_right_joint`
  - `back_left_joint`
  - `back_right_joint`
- **Kinematics**: Configured for base frame offset and wheel separation

#### 3. **Lift Position Controller**
- **Type**: `joint_trajectory_controller/JointTrajectoryController`
- **Function**: Controls vertical lift mechanism
- **Use Case**: Position lift at specific heights for scanning

### Twist Multiplexer (`config/twist_mux.yaml`)

Prioritizes command velocity sources:
1. **Autonomous Navigation** (Priority 10)
2. **Joystick Teleoperation** (Priority 100)
3. **Keyboard Teleoperation** (Priority 100)

---

## Important Scripts and Nodes

### 1. **`rack_waypoints.py`**
- **Purpose**: Records waypoints from RViz for autonomous navigation
- **Input**: `/recorded_pose` (PoseStamped from RViz "Publish 2D goal" tool)
- **Output**: `recorded_waypoints.yaml`
- **Usage**:
  ```bash
  python3  ~/inter_iit_ws/src/mecanum_description/scripts/rack_waypoints.py
  # Use RViz "Publish 2D goal" tool to record pose for each rack
  ```

### 6. **`return_to_spawn.py`**
- **Purpose**: Resets robot to spawn position in simulation
- **Use Case**: Quick reset during testing; Change teh spawn position in the code (Default should also work)


## Configuration Files

### `controllers.yaml`
Defines all ROS 2 Control interfaces:
- Controller manager parameters
- Joint state broadcaster
- Mecanum drive controller with kinematics
- Lift trajectory controller

### `twist_mux.yaml`
Configures command velocity prioritization:
```yaml
topics:
  - name: autonomous
    topic: /cmd_vel_nav
    priority: 10
  - name: joystick
    topic: /cmd_vel_joy
    priority: 100
```

---

## RViz Configurations

### 1. **`cartographer.rviz`**
- Optimized for Cartographer SLAM
- Displays submaps, trajectory, and laser scans

### 2. **`nav2.rviz`**
- Standard Nav2 visualization
- Shows global/local costmaps, paths, and poses

---

## Dependencies

### Build Dependencies
- `ament_cmake`
- `urdf`
- `xacro`
- `robot_state_publisher`

### Runtime Dependencies
- `ros2_control`
- `ros2_controllers`
- `mecanum_drive_controller`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `gazebo_ros` (for simulation)
- `nav2_bringup` (for navigation)


## Integration with Other Packages

### With `mecanum_bringup` (Hardware)
```bash
ros2 launch mecanum_bringup hardware.launch.py
```

### With `mecanum_navigation_setup` (SLAM)
```bash
ros2 launch mecanum_navigation_setup cartographer.launch.py 
```


### With `mecanum_navigation_setup` (Nav2)
```bash
ros2 launch mecanum_navigation_setup cartographer_navigation.launch.py
```



### With `warehouse_rover_mission_control` (Autonomous Operation)
```bash
ros2 launch warehouse_rover_mission_control autonomous_inventory.launch.py
```

---

## Troubleshooting

### Issue: Robot not moving in simulation
**Solution**: Check that `mecanum_drive_controller` is loaded and active:
```bash
ros2 control list_controllers
```

### Issue: IMU data not working with EKF
**Solution**: Use `sensor_bridge.py` to fix covariance:
```bash
ros2 run mecanum_description sensor_bridge.py
```

### Issue: Lift not responding
**Solution**: Verify lift controller is active:
```bash
ros2 control list_controllers | grep lift
```

---
