# Mecanum Hardware Package

ROS2 hardware interface for mecanum robot with NEMA23 stepper motors, ESP32 controllers, and IMU sensor fusion.

## Overview

This package provides the **ros2_control** hardware interface for controlling a mecanum wheel robot with:
- 4× NEMA23 stepper motors
- 2× ESP32 microcontrollers (serial communication)
- IMU sensor (I2C connection)

## Package Structure

```
mecanum_hardware/
├── config/
│   ├── hardware_controllers.yaml    # Controller config (50 Hz)
│   ├── lidar.yaml                    # RPLidar settings
│   ├── imu.yaml                      # IMU filter parameters
│   └── camera.yaml                   # RealSense camera (optional)
├── firmware/
│   └── esp32_stepper_controller/    # Arduino firmware for ESP32
│       └── esp32_stepper_controller.ino
├── include/mecanum_hardware/
│   └── mecanum_stepper_interface.hpp # Hardware interface header
├── launch/
│   ├── hardware.launch.py            # Base hardware + sensors
│   ├── hardware_mapping.launch.py    # Mapping mode
│   └── hardware_navigation.launch.py # Navigation mode
├── scripts/
│   └── imu_publisher_node.py         # IMU data publisher
├── src/
│   └── mecanum_stepper_interface.cpp # Hardware interface implementation
└── CMakeLists.txt
```

## Hardware Interface

### Class: `MecanumStepperInterface`

Implements `hardware_interface::SystemInterface` for ros2_control.

**Features**:
- Dual ESP32 serial communication (115200 baud)
- IMU sensor fusion (complementary filter)
- Wheel odometry calculation
- Slip detection (IMU vs wheel discrepancy)
- Hardware lifecycle management

**Control Loop**: 50 Hz update rate

**Topics Published**:
- `/mecanum_drive_controller/odom` (nav_msgs/Odometry) - Fused odometry

**Topics Subscribed**:
- `/imu/data` (sensor_msgs/Imu) - Raw IMU data (RELIABLE QoS)

### ESP32 Communication Protocol

**Command Format** (Hardware Interface → ESP32):
```
V,<steps_per_sec_motor1>,<steps_per_sec_motor2>\n
```

**Example**:
```
V,400,-300\n  # Motor 1: 400 steps/s forward, Motor 2: 300 steps/s backward
```

**Status Request**:
```
STATUS\n
```

**Response Format** (ESP32 → Hardware Interface):
```
ESP<id>:M1=<steps1>,M2=<steps2>\n
```

**Emergency Stop**:
```
STOP\n
```

### ESP32 Assignment

| ESP32 | Motors | Serial Port | Firmware Setting |
|-------|--------|-------------|------------------|
| ESP32 #1 | Front Left + Front Right | `/dev/ttyUSB0` | `ESP_ID = 1` |
| ESP32 #2 | Back Left + Back Right | `/dev/ttyUSB1` | `ESP_ID = 2` |

## Configuration

### hardware_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

mecanum_drive_controller:
  ros__parameters:
    # Wheel parameters
    wheel_radius: 0.05                    # meters
    wheel_separation_x: 0.25              # meters (front-back)
    wheel_separation_y: 0.30              # meters (left-right)
    
    # Velocity limits
    linear.x.max_velocity: 0.5            # m/s
    linear.y.max_velocity: 0.5            # m/s
    angular.z.max_velocity: 1.0           # rad/s
    
    # Acceleration limits
    linear.x.max_acceleration: 0.5        # m/s²
    linear.y.max_acceleration: 0.5        # m/s²
    angular.z.max_acceleration: 1.0       # rad/s²
    
    # Safety
    cmd_vel_timeout: 0.5                  # seconds
```

### URDF Parameters

Defined in `mec_rob.ros2_control.hardware.xacro`:

```xml
<param name="esp1_port">/dev/ttyUSB0</param>
<param name="esp2_port">/dev/ttyUSB1</param>
<param name="steps_per_revolution">200</param>
<param name="wheel_radius">0.05</param>
<param name="gear_ratio">1.0</param>
<param name="wheel_separation_x">0.25</param>
<param name="wheel_separation_y">0.30</param>
<param name="use_imu">true</param>
```

## Launch Files

### 1. hardware.launch.py

**Purpose**: Launch base hardware interface with all sensors

**Usage**:
```bash
ros2 launch mecanum_hardware hardware.launch.py \
  esp1_port:=/dev/ttyUSB0 \
  esp2_port:=/dev/ttyUSB1 \
  lidar_port:=/dev/ttyUSB2 \
  use_depth_camera:=false
```

**Nodes Launched**:
- `robot_state_publisher` - Publishes robot TF tree from URDF
- `controller_manager` - ros2_control manager
- `joint_state_broadcaster` - Publishes joint states
- `mecanum_drive_controller` - Mecanum kinematics controller
- `rplidar_node` - LiDAR driver
- `imu_filter_madgwick_node` - IMU orientation filter
- `imu_publisher_node` - Raw IMU data from GPIO
- `realsense2_camera_node` - Depth camera (if enabled)

**Launch Sequence**:
- t=0s: Robot state publisher + ros2_control
- t=2s: Controllers (broadcaster, mecanum)
- t=4s: Sensors (LiDAR, IMU filter)
- t=5s: IMU publisher, camera

### 2. hardware_mapping.launch.py

**Purpose**: SLAM mapping mode to build environment maps

**Usage**:
```bash
ros2 launch mecanum_hardware hardware_mapping.launch.py \
  esp1_port:=/dev/ttyUSB0 \
  esp2_port:=/dev/ttyUSB1 \
  lidar_port:=/dev/ttyUSB2
```

**Additional Nodes**:
- `cartographer_node` - SLAM with sensor fusion
- `cartographer_occupancy_grid_node` - Generates occupancy grid map

**Cartographer Config**: `cartographer_mapping.lua`
- Publishes: `map → odom → base_link`
- Fuses: IMU + wheel odometry + LiDAR scan

**TF Tree**:
```
map → odom → base_link → [sensors, wheels]
└─── All published by Cartographer ────┘
```

**Save Map**:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 3. hardware_navigation.launch.py

**Purpose**: Autonomous navigation with saved map

**Usage**:
```bash
ros2 launch mecanum_hardware hardware_navigation.launch.py \
  map:=/path/to/my_map.yaml \
  esp1_port:=/dev/ttyUSB0 \
  esp2_port:=/dev/ttyUSB1 \
  lidar_port:=/dev/ttyUSB2
```

**Additional Nodes**:
- `cartographer_node` - Odometry sensor fusion
- `amcl` - Monte Carlo localization on saved map
- `map_server` - Serves saved map
- `planner_server` - Global path planning
- `controller_server` - Local trajectory control
- `recoveries_server` - Recovery behaviors
- `bt_navigator` - Behavior tree navigation

**Cartographer Config**: `cartographer_odom.lua`
- Publishes: `odom → base_link` only
- AMCL publishes: `map → odom`

**TF Tree**:
```
map → odom → base_link → [sensors, wheels]
(AMCL) (Cartographer)
```

## IMU Publisher Node

**Script**: `scripts/imu_publisher_node.py`

**Purpose**: Read IMU data from I2C GPIO and publish to ROS2

**Supported IMUs**:
- MPU6050 (I2C address: 0x68)
- BNO055 (I2C address: 0x28)

**Topic**: `/imu/data_raw` (sensor_msgs/Imu)

**Frequency**: 100 Hz

**I2C Connection**:
- SDA: GPIO 2 (Pin 3)
- SCL: GPIO 3 (Pin 5)
- VCC: 3.3V (Pin 1)
- GND: GND (Pin 6)

**Enable I2C**:
```bash
sudo raspi-config
# Interface Options → I2C → Enable
sudo reboot
```

**Test I2C**:
```bash
i2cdetect -y 1
# Should show 0x68 (MPU6050) or 0x28 (BNO055)
```

**Dependencies**:
```bash
pip3 install adafruit-circuitpython-mpu6050
pip3 install adafruit-circuitpython-bno055
```

## ESP32 Firmware

**Location**: `firmware/esp32_stepper_controller/esp32_stepper_controller.ino`

**Flash Instructions**:
1. Install Arduino IDE with ESP32 board support
2. Open `esp32_stepper_controller.ino`
3. Set `ESP_ID = 1` for front motors, `ESP_ID = 2` for back motors
4. Configure pin mappings if needed
5. Upload to ESP32

**Pin Configuration** (Default):

**Motor 1** (Left):
- STEP: GPIO 12
- DIR: GPIO 14
- ENABLE: GPIO 27

**Motor 2** (Right):
- STEP: GPIO 26
- DIR: GPIO 25
- ENABLE: GPIO 33

**Features**:
- Acceleration limiting (2000 steps/s²)
- Watchdog timer (500ms timeout)
- Emergency stop support
- Status reporting

## Testing

### Test Hardware Interface
```bash
# Launch hardware
ros2 launch mecanum_hardware hardware.launch.py

# Check controllers
ros2 control list_controllers

# Should show:
# joint_state_broadcaster [active]
# mecanum_drive_controller [active]

# Test movement (wheels off ground!)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1}}' --rate 1
```

### Test ESP32 Communication
```bash
# Find ESP32 ports
ls -l /dev/ttyUSB*

# Test ESP1
echo "STATUS" > /dev/ttyUSB0
cat /dev/ttyUSB0

# Should respond: ESP1:M1=0,M2=0
```

### Test IMU
```bash
# Check I2C
i2cdetect -y 1

# Launch IMU publisher
ros2 run mecanum_hardware imu_publisher_node.py

# Monitor IMU data
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw  # Should be ~100 Hz
```

## Troubleshooting

### Problem: Controllers not loading
**Solution**: Check USB port permissions
```bash
sudo chmod 666 /dev/ttyUSB*
```

### Problem: IMU not detected
**Solution**: Enable I2C and check wiring
```bash
sudo raspi-config  # Enable I2C
i2cdetect -y 1     # Verify detection
```

### Problem: Robot doesn't move
**Solution**: 
1. Check ESP32 power supply
2. Verify firmware is flashed with correct ESP_ID
3. Test ESP32 serial communication
4. Check motor driver enable pins

### Problem: Odometry drift
**Solution**: 
1. Enable IMU fusion: `use_imu: true` in URDF
2. Calibrate IMU orientation

### Problem: Nav2 doesn't start
**Solution**: Increase launch delays if Cartographer isn't ready
```python
# In hardware_navigation.launch.py
TimerAction(period=12.0, ...)  # Increase from 12s to 15s
```

## Dependencies

**ROS2 Packages**:
- `ros2_control`
- `ros2_controllers`
- `controller_manager`
- `hardware_interface`
- `cartographer_ros`
- `nav2_bringup`
- `rplidar_ros`
- `imu_filter_madgwick`
- `realsense2_camera` (optional)

**Python Libraries**:
- `adafruit-circuitpython-mpu6050`
- `adafruit-circuitpython-bno055`

**System**:
- Docker with privileged mode (hardware access)
- I2C enabled (`raspi-config`)
- USB serial ports accessible

---

**Author**: Mecanum Robot Team  
**Last Updated**: November 2025  
**ROS2 Version**: Humble Hawksbill
