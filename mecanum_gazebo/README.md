# Mecanum Gazebo Package

Gazebo simulation package for the mecanum wheel robot with ROS2 Humble integration.

## Overview

This package provides a complete Gazebo simulation environment for testing the mecanum robot before hardware deployment. It includes physics simulation, sensor models, ros2_control integration, and a custom warehouse arena with racks, walls, and navigation tapes.


**Note** The paths to all meshes in the world file are absolute wrt docker image provided for this repo. Hence, if you want to use this repo in your environment, update the paths

## Quick Start

# Launch the simulation
ros2 launch mecanum_gazebo simulation_world.launch.py

```

## Package Structure

```
mecanum_gazebo/
├── arena/                         # Custom warehouse world
│   ├── arena.world               # Main Gazebo world file
│   ├── materials/                # Textures and materials
│   ├── models/                   # Custom Gazebo models
│   ├── racks/                    # Warehouse rack models
│   ├── tapes/                    # Navigation tape models
│   └── walls/                    # Wall models
├── launch/                        # Launch files
│   ├── simulation_world.launch.py # Main simulation launcher
│   ├── gazebo.launch.py          # Alternative Gazebo launcher
│   ├── spawn.launch.py           # Robot spawner
│   ├── controller.launch.py      # Controller loader
│   ├── robot_state.launch.py    # Robot state publisher
│   ├── joint_broadcast.launch.py # Joint state broadcaster
│   ├── display.launch.py         # RViz display
│   └── urdf.rviz                 # RViz configuration
├── include/                       # Header files (if any)
├── CMakeLists.txt                # Build configuration
├── package.xml                   # Package dependencies
├── LICENSE                       # GPL-2.0 license
└── README.md                     # This file
```

**Note**: Robot URDF files are located in the `mecanum_description` package under `urdf/` directory.

## Robot Description (URDF)

### Main URDF Location

Robot URDF files are located in the **mecanum_description** package:
- Main robot: `mecanum_description/urdf/mec_rob.xacro`
- Simulation control: `mecanum_description/urdf/mec_rob.ros2_control.xacro`
- Hardware control: `mecanum_description/urdf/mec_rob.ros2_control.hardware.xacro`
- Sensors: `lidar.xacro`, `depth_camera.xacro`, `imu.xacro`, `lift_camera.xacro`, `lift.xacro`
- Gazebo plugins: `*.gazebo` files



**What happens**:
1. Gazebo GUI opens with warehouse arena (racks, walls, navigation tapes)
2. Robot spawns at specified position (default: x=0.8, y=0, z=0.055)
3. Controllers load (takes ~10 seconds):
   - `joint_state_broadcaster` - Publishes joint states
   - `mecanum_drive_controller` - Mecanum kinematics
4. Sensors start publishing data (LiDAR, IMU, cameras)
5. Robot is ready for teleoperation or autonomous navigation

```

**2. Sensor Plugins**:
- **LiDAR**: `libgazebo_ros_ray_sensor.so`
  - Topic: `/scan`
  - Rate: 10 Hz
  - Range: 0.15-12.0 m
  
- **IMU**: `libgazebo_ros_imu_sensor.so`
  - Topic: `/imu/data`
  - Rate: 100 Hz
  - Outputs: Linear acceleration, angular velocity, orientation
  
- **Depth Camera**: `libgazebo_ros_camera.so`
  - Topics: `/camera/color/image_raw`, `/camera/depth/image_raw`
  - Resolution: 640×480
  - FPS: 30

## Controllers Configuration

### File Location

Controllers are configured in the **mecanum_description** package:
- File: `mecanum_description/config/controllers.yaml`


### Controller Types

**Joint State Broadcaster**:
- Publishes `/joint_states` topic
- Used by `robot_state_publisher` for TF tree

**Mecanum Drive Controller**:
- Subscribes: `/mecanum_drive_controller/reference` (TwistStamped)
- Publishes: `/mecanum_drive_controller/odom` (Odometry)
- Computes mecanum wheel inverse kinematics
- Publishes `odom → base_link` TF

## Teleoperation

### Keyboard Control

Teleoperation is provided by the **mecanum_teleop** package:

```bash
ros2 run mecanum_teleop mecanum_teleop_key
```


## Warehouse Arena

The simulation includes a custom warehouse environment located in the `arena/` directory:

### Arena Components

- **arena.world**: Main Gazebo world file with warehouse setup
- **racks/**: Warehouse storage rack models for navigation
- **walls/**: Perimeter and internal wall models
- **tapes/**: Navigation line markers on the floor
- **materials/**: Textures and visual materials
- **models/**: Custom Gazebo model definitions

### Arena Features

- Realistic warehouse layout with storage racks
- Navigation tapes for line-following algorithms
- Proper lighting and shadows
- Physics-enabled collision models
- Optimized for both visualization and navigation testing

## Topics Reference

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | 50 Hz | Wheel joint states |
| `/mecanum_drive_controller/odom` | nav_msgs/Odometry | 50 Hz | Raw wheel odometry |
| `/odom` | nav_msgs/Odometry | 50 Hz | Processed odometry |
| `/odometry/local` | nav_msgs/Odometry | 50 Hz | EKF fused (if running) |
| `/imu/data` | sensor_msgs/Imu | 100 Hz | Raw IMU |
| `/imu/data_fixed` | sensor_msgs/Imu | 100 Hz | IMU with covariances |
| `/scan` | sensor_msgs/LaserScan | 10 Hz | LiDAR scan |
| `/camera/color/image_raw` | sensor_msgs/Image | 30 Hz | RGB image |
| `/camera/depth/image_raw` | sensor_msgs/Image | 30 Hz | Depth image |
| `/tf` | tf2_msgs/TFMessage | Variable | Transform tree |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (input) |
| `/mecanum_drive_controller/reference` | geometry_msgs/TwistStamped | Controller input |

## TF Tree

### Simulation TF Tree

```
odom
 └─ base_link (published by mecanum_drive_controller)
     ├─ imu_link
     ├─ lidar_link
     ├─ camera_link
     ├─ wheel_frame_FL_1
     ├─ wheel_frame_FR_1
     ├─ wheel_frame_BL_1
     └─ wheel_frame_BR_1
         └─ roller_* (36 rollers total)
```

**Publishers**:
- `odom → base_link`: `mecanum_drive_controller`
- `base_link → sensors/wheels`: `robot_state_publisher` (from URDF)