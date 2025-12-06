# Mecanum Gazebo Package

## Overview

This package provides a comprehensive, pre-configured simulation environment for testing and developing the mecanum robot prior to hardware deployment.

Key features include:

  * **Physics Simulation** with **ros2\_control** integration.
  * **Sensor Models** (LiDAR, IMU, Depth Camera).
  * Custom **Warehouse Arena** (`arena.world`) with racks, walls, and navigation tapes.

> **Note on Mesh Paths**: The paths to all meshes in the world file are **absolute** with respect to the provided Docker image. If you use this repository in your own environment, you **must update these mesh paths** within the world file.

-----

## Quick Start

Launch the complete simulation environment:

```bash
ros2 launch mecanum_gazebo simulation_world.launch.py
```

### Simulation Startup Sequence

1.  **Gazebo GUI** opens, displaying the custom warehouse arena (racks, walls, navigation tapes).
2.  The robot spawns at the specified position (default: $x=0.8, y=0, z=0.055$).
3.  Controllers load ($\sim 10$ seconds):
      * `joint_state_broadcaster`: Publishes joint states.
      * `mecanum_drive_controller`: Implements **Mecanum kinematics**.
4.  Sensors begin publishing data (LiDAR, IMU, cameras).
5.  The robot is ready for teleoperation or autonomous navigation.

-----

## Package Structure

The repository is organized to clearly separate world assets and configuration files.

```
mecanum_gazebo/
├── arena/                       # Custom warehouse world assets
│   ├── arena.world              # Main Gazebo world file
│   ├── materials/               # Textures and materials
│   ├── models/                  # Custom Gazebo models
│   ├── racks/                   # Warehouse rack models
│   ├── tapes/                   # Navigation tape models
│   └── walls/                   # Wall models
├── launch/                      # ROS 2 Launch files
│   ├── simulation_world.launch.py # Main simulation launcher
│   ├── gazebo.launch.py         # Alternative Gazebo launcher
│   ├── spawn.launch.py          # Robot spawner
│   ├── controller.launch.py     # Controller loader
│   ├── robot_state.launch.py    # Robot state publisher
│   ├── joint_broadcast.launch.py# Joint state broadcaster
│   ├── display.launch.py        # RViz display
│   └── urdf.rviz                # RViz configuration
├── include/                     # Header files (if any)
├── CMakeLists.txt               # Build configuration
├── package.xml                  # Package dependencies
├── LICENSE                      # GPL-2.0 license
└── README.md                    # This file
```

> **Note**: Robot URDF files are located in the separate `mecanum_description` package under the `urdf/` directory.

-----

## Robot Description (URDF)

Robot URDF files are located in the **mecanum\_description** package.

### Main URDF Location

| Component | File Path (in `mecanum_description/urdf/`) |
| :--- | :--- |
| **Main Robot Structure** | `mec_rob.xacro` |
| **Simulation Control** | `mec_rob.ros2_control.xacro` |
| **Hardware Control** | `mec_rob.ros2_control.hardware.xacro` |
| **Sensors** | `lidar.xacro`, `depth_camera.xacro`, `imu.xacro`, `lift_camera.xacro`, `lift.xacro` |
| **Gazebo Plugins** | `*.gazebo` files |

### Sensor Plugin Specifications

Gazebo plugins enable realistic sensor data publishing:

| Sensor | Plugin Library | Topic | Rate | Key Output/Range |
| :--- | :--- | :--- | :--- | :--- |
| **LiDAR** | `libgazebo_ros_ray_sensor.so` | `/scan` | 10 Hz | Range: $0.15 \text{-} 12.0 \text{ m}$ |
| **IMU** | `libgazebo_ros_imu_sensor.so` | `/imu/data` | 100 Hz | Linear acceleration, angular velocity, orientation |
| **Depth Camera** | `libgazebo_ros_camera.so` | `/camera/color/image_raw`, `/camera/depth/image_raw` | 30 FPS | Resolution: $640 \times 480$ |

-----

## Controllers Configuration

Controllers are configured via the `mecanum_description/config/controllers.yaml` file.

### Controller Types

| Controller | Subscribed Topic | Published Topic | Function |
| :--- | :--- | :--- | :--- |
| **Joint State Broadcaster** | N/A | `/joint_states` | Publishes joint states used by `robot_state_publisher` for the TF tree. |
| **Mecanum Drive Controller** | `/mecanum_drive_controller/reference` (`TwistStamped`) | `/mecanum_drive_controller/odom` (`Odometry`) | Computes **mecanum wheel inverse kinematics** and publishes the `odom → base_link` TF. |

-----

## Teleoperation

Keyboard control is provided by the **mecanum\_teleop** package:

```bash
ros2 run mecanum_teleop mecanum_teleop_key
```

-----

## Warehouse Arena

The custom simulation environment is located in the `arena/` directory.

### Arena Components and Features

| Component | Files | Features |
| :--- | :--- | :--- |
| **Main World** | `arena.world` | Defines the overall warehouse setup, lighting, and physics. |
| **Models** | `racks/`, `walls/`, `models/` | Provides realistic storage racks, perimeter walls, and custom objects for navigation. |
| **Navigation** | `tapes/` | Includes navigation line markers on the floor for testing **line-following algorithms**. |

-----

## Topics Reference

### Published Topics

| Topic | Type | Rate | Description |
| :--- | :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/JointState` | 50 Hz | Wheel joint states. |
| `/mecanum_drive_controller/odom` | `nav_msgs/Odometry` | 50 Hz | Raw wheel odometry from the controller. |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Processed odometry. |
| `/odometry/local` | `nav_msgs/Odometry` | 50 Hz | EKF fused odometry (if running). |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | Raw IMU data. |
| `/imu/data_fixed` | `sensor_msgs/Imu` | 100 Hz | IMU data with covariances. |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | LiDAR scan data. |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | RGB image. |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | Depth image. |
| `/tf` | `tf2_msgs/TFMessage` | Variable | Transform tree broadcasts. |

### Subscribed Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (input). |
| `/mecanum_drive_controller/reference` | `geometry_msgs/TwistStamped` | Controller input. |

-----

## TF Tree

### Simulation TF Tree

The structure of the transform tree:

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

  * `odom → base_link`: `mecanum_drive_controller`
  * `base_link → sensors/wheels`: `robot_state_publisher` (from URDF)
