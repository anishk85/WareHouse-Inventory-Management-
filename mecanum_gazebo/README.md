# Mecanum in Gazebo Package

Gazebo simulation package for the mecanum wheel robot with ROS2 Humble integration.

## Overview

This package provides a complete Gazebo simulation environment for testing the mecanum robot before hardware deployment. It includes physics simulation, sensor models, and ros2_control integration.

## Package Structure

```
mecanum_gazebo/
├── config/
│   ├── controllers.yaml          # Simulation controller config
│   └── ekf.yaml                   # Robot localization (optional)
├── launch/
│   ├── gazebo.launch.py          # Main simulation launcher
│   └── ekf.launch.py              # EKF sensor fusion (optional)
├── meshes/                        # 3D meshes for visualization
│   ├── base_link.stl
│   ├── wheel_frame_*.stl
│   └── roller_*.stl
├── scripts/
│   └── mecanum_teleop_key.py     # Keyboard teleoperation
├── urdf/
│   ├── mec_rob.xacro             # Main robot description
│   ├── mec_rob.ros2_control.xacro     # Simulation control
│   ├── mec_rob.ros2_control.hardware.xacro  # Hardware control
│   ├── mec_rob.gazebo            # Gazebo plugins
│   ├── mec_rob.trans             # Joint transformations
│   ├── lidar.xacro               # LiDAR sensor
│   ├── lidar.gazebo              # LiDAR Gazebo plugin
│   ├── depth_camera.xacro        # RealSense camera
│   ├── depth_camera.gazebo       # Camera Gazebo plugin
│   ├── imu.xacro                 # IMU sensor
│   ├── imu.gazebo                # IMU Gazebo plugin
│   └── materials.xacro           # Visual materials
├── worlds/
│   └── empty.world               # Gazebo world file
└── CMakeLists.txt
```

## Robot Description (URDF)

### Main URDF: `mec_rob.xacro`

The robot URDF supports **conditional compilation** for simulation vs hardware:

```xml
<!-- Arguments -->
<xacro:arg name="use_sim" default="true"/>
<xacro:arg name="use_depth_camera" default="true"/>

<!-- Simulation mode -->
<xacro:if value="$(arg use_sim)">
  <xacro:include filename="mec_rob.ros2_control.xacro"/>
  <xacro:include filename="lidar.gazebo"/>
  <xacro:include filename="depth_camera.gazebo"/>
  <xacro:include filename="imu.gazebo"/>
</xacro:if>

<!-- Hardware mode -->
<xacro:unless value="$(arg use_sim)">
  <xacro:include filename="mec_rob.ros2_control.hardware.xacro"/>
</xacro:unless>
```

**Usage**:
```bash
# Simulation
ros2 launch mecanum_gazebo gazebo.launch.py use_sim:=true

# Hardware
ros2 launch mecanum_hardware hardware.launch.py use_sim:=false
```

### Robot Specifications

**Base Dimensions**:
- Length: 0.30 m
- Width: 0.40 m
- Height: 0.15 m
- Mass: 19.35 kg

**Wheels**:
- Type: Mecanum (with rollers)
- Radius: 0.05 m (50 mm)
- Width: 0.05 m
- Configuration: 4-wheel independent (FL, FR, BL, BR)
- Joint Type: Continuous rotation

**Wheel Separation**:
- X-axis (front-back): 0.25 m
- Y-axis (left-right): 0.30 m

**Sensors**:
- LiDAR: RPLidar A1 (360°, 0-12m range)
- IMU: Simulated 6-DOF (100 Hz)
- Depth Camera: RealSense D435 (640×480, optional)

## Gazebo Simulation

### Launch Gazebo

```bash
# Basic launch
ros2 launch mecanum_gazebo gazebo.launch.py

# With depth camera disabled
ros2 launch mecanum_gazebo gazebo.launch.py use_depth_camera:=false

# Pause at start
ros2 launch mecanum_gazebo gazebo.launch.py pause:=true
```

**What happens**:
1. Gazebo GUI opens with empty world
2. Robot spawns at origin (0, 0, 0)
3. Controllers load (takes ~10 seconds):
   - `joint_state_broadcaster` - Publishes joint states
   - `mecanum_drive_controller` - Mecanum kinematics
4. `cmd_vel_bridge` starts - Converts Twist → TwistStamped
5. Sensors start publishing data

### Gazebo Plugins

**1. Differential Drive Plugin** (disabled, using ros2_control):
```xml
<!-- In mec_rob.gazebo -->
<!-- libgazebo_ros_diff_drive.so - NOT USED -->
```

**2. ros2_control Plugin** (active):
```xml
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
  <parameters>$(find mecanum_gazebo)/config/controllers.yaml</parameters>
</plugin>
```

**3. Sensor Plugins**:
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

### File: `config/controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz (simulation can handle higher rate)

joint_state_broadcaster:
  ros__parameters:
    publish_rate: 50.0

mecanum_drive_controller:
  ros__parameters:
    # Joint names
    front_left_wheel_joint: front_left_joint
    front_right_wheel_joint: front_right_joint
    back_left_wheel_joint: back_left_joint
    back_right_wheel_joint: back_right_joint
    
    # Kinematics
    wheel_radius: 0.05                # meters
    wheel_separation_x: 0.25          # front-back distance
    wheel_separation_y: 0.30          # left-right distance
    
    # TF Publishing
    enable_odom_tf: true              # Publish odom → base_link
    odom_frame_id: odom
    base_frame_id: base_link
    
    # Velocity limits
    linear.x.max_velocity: 1.0        # m/s
    linear.y.max_velocity: 1.0        # m/s
    angular.z.max_velocity: 2.0       # rad/s
```

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

```bash
ros2 run mecanum_gazebo mecanum_teleop_key.py
```

**Controls**:
```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

u/o: Forward + rotate
i  : Forward
j/l: Rotate left/right
k  : Stop
m/.: Backward + rotate
,  : Backward

q/z: Increase/decrease max speeds by 10%
w/x: Increase/decrease only linear speed by 10%
e/c: Increase/decrease only angular speed by 10%

CTRL-C to quit
```

**Current Speed Display**:
```
currently:	speed 0.5	turn 1.0
```

### Manual Command Publishing

```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}' -r 1

# Strafe right
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {y: 0.3}}' -r 1

# Rotate counterclockwise
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}' -r 1

# Diagonal movement (forward-right + rotate)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3, y: 0.2}, angular: {z: 0.3}}' -r 1
```

## cmd_vel_bridge Node

**Purpose**: Convert `Twist` messages to `TwistStamped` for the controller

**Source**: Built-in node in launch file

```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='cmd_vel_bridge',
    arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
    remappings=[
        ('/cmd_vel', '/mecanum_drive_controller/reference')
    ]
)
```

**Data Flow**:
```
/cmd_vel (Twist) → cmd_vel_bridge → /mecanum_drive_controller/reference (TwistStamped)
```

## Robot Localization (EKF)

### Optional Sensor Fusion

**Launch**:
```bash
ros2 launch mecanum_gazebo ekf.launch.py
```

**Config**: `config/ekf.yaml`

**Purpose**: Fuses wheel odometry + IMU for better state estimation

**Settings**:
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    publish_tf: false  # Disabled to avoid conflict with controller
    
    # Input sources
    odom0: /mecanum_drive_controller/odometry
    odom0_config: [false, false, false,  # x, y, z position
                   false, false, false,  # roll, pitch, yaw
                   true,  true,  false,  # x_vel, y_vel, z_vel
                   false, false, true,   # roll_vel, pitch_vel, yaw_vel
                   false, false, false]  # x_accel, y_accel, z_accel
    
    imu0: /imu/data_fixed
    imu0_config: [false, false, false,  # position
                  false, false, true,   # orientation (yaw only)
                  false, false, false,  # linear velocity
                  false, false, true,   # angular velocity (yaw rate)
                  true,  true,  false]  # linear acceleration (x, y)
```

**Output**: `/odometry/local` (Fused odometry)

**Note**: Currently `publish_tf: false` to avoid TF conflicts with `mecanum_drive_controller`.

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

## Monitoring & Debugging

### Check System Status

```bash
# List nodes
ros2 node list

# Check controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster  [active]
# mecanum_drive_controller [active]

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check topics
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /mecanum_drive_controller/odom

# Echo odometry
ros2 topic echo /mecanum_drive_controller/odom

# Monitor transforms
ros2 run tf2_ros tf2_echo odom base_link
```

### RViz Visualization

```bash
# Launch RViz
rviz2

# Add displays:
# - RobotModel (shows URDF)
# - TF (transform tree)
# - LaserScan (/scan)
# - Odometry (/mecanum_drive_controller/odom)
# - Image (/camera/color/image_raw)
```

**RViz Configuration**:
- Fixed Frame: `odom`
- Robot Description: `/robot_description`

## Troubleshooting

### Problem: Robot doesn't move
**Causes**:
1. Controllers not active
2. cmd_vel_bridge not running
3. Wrong topic name

**Solutions**:
```bash
# Check controller status
ros2 control list_controllers

# Restart controllers if needed
ros2 control set_controller_state mecanum_drive_controller stop
ros2 control set_controller_state mecanum_drive_controller start

# Check if bridge is running
ros2 node list | grep bridge

# Verify topic connection
ros2 topic info /cmd_vel
ros2 topic info /mecanum_drive_controller/reference
```

### Problem: Gazebo crashes on launch
**Solution**: Increase Gazebo timeout
```python
# In gazebo.launch.py
gzserver_cmd = IncludeLaunchDescription(
    ...
    launch_arguments={'timeout': '60.0'}.items()  # Increase from default
)
```

### Problem: Controllers timeout
**Solution**: Increase controller load delay
```python
# In gazebo.launch.py
TimerAction(
    period=10.0,  # Increase from 2.0
    actions=[load_joint_state_broadcaster]
)
```

### Problem: TF tree errors
**Solution**: Check for multiple TF publishers
```bash
# Find TF publishers
ros2 topic info /tf

# Check EKF settings
# Set publish_tf: false in ekf.yaml if using controller TF
```

### Problem: Wheels slide in Gazebo
**Solution**: Adjust friction in URDF
```xml
<gazebo reference="wheel_frame_FL_1">
  <mu1>1.0</mu1>  <!-- Increase friction -->
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

## Testing Workflow

### 1. Basic Movement Test
```bash
# Terminal 1: Launch Gazebo
ros2 launch mecanum_gazebo gazebo.launch.py

# Terminal 2: Test keyboard control
ros2 run mecanum_gazebo mecanum_teleop_key.py

# Try all directions: forward, back, strafe, rotate, diagonal
```

### 2. Sensor Test
```bash
# Terminal 1: Gazebo

# Terminal 2: Monitor sensors
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /camera/color/image_raw

# Terminal 3: Visualize in RViz
rviz2
```

### 3. Odometry Test
```bash
# Drive in square pattern
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}' -r 1
# Wait 2 seconds, stop, rotate 90°
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 1.57}}' -r 1
# Repeat 4 times

# Check drift
ros2 topic echo /mecanum_drive_controller/odom
```

## Integration with Navigation

This simulation package integrates with:
- **Cartographer**: SLAM mapping
- **Nav2**: Autonomous navigation
- **AMCL**: Localization

See [mecanum_navigation_setup](../mecanum_navigation_setup/README.md) package for navigation configs.

## Related Documentation

- [Main README](../../README.md) - Project overview
- [mecanum_hardware](../mecanum_hardware/README.md) - Real hardware interface
- [mecanum_navigation_setup](../mecanum_navigation_setup/README.md) - Navigation configs
- [ARCHITECTURE.md](../../ARCHITECTURE.md) - System architecture

## Dependencies

**ROS2 Packages**:
- `gazebo_ros_pkgs`
- `ros2_control`
- `ros2_controllers`
- `controller_manager`
- `robot_state_publisher`
- `joint_state_publisher`
- `xacro`
- `robot_localization` (optional)

**System**:
- Gazebo 11 (comes with ROS2 Humble desktop)
- Ubuntu 22.04

---

**Author**: Mecanum Robot Team  
**Last Updated**: November 2025  
**ROS2 Version**: Humble Hawksbill
