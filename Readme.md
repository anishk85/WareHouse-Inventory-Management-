# Inter-IIT Warehouse Robot Workspace

This ROS 2 workspace contains the software stack for a mecanum-wheeled mobile robot designed for autonomous warehouse inventory management. The system includes simulation, hardware interfacing, navigation, and specific applications for QR code scanning and rack detection.

## Workspace Structure

The workspace is organized into core robot packages and warehouse-specific application packages.

### Core Robot Packages

*   **`mecanum_hardware`** (located in `mecanum_bringup`):
    *   Provides the hardware interface for the robot, controlling stepper motors and reading sensors.
    *   Handles communication with the ESP32 firmware.
*   **`mecanum_description`**:
    *   Contains the URDF and Xacro robot descriptions.
    *   Defines the physical properties, visual meshes, and collision models.
*   **`mecanum_gazebo`**:
    *   Simulation environment for Gazebo.
    *   Includes worlds and launch files to spawn the robot in a simulated warehouse.
*   **`mecanum_navigation_setup`**:
    *   Configuration and launch files for the ROS 2 Navigation Stack (Nav2).
    *   Includes SLAM Toolbox and Cartographer for mapping and localization.
*   **`mecanum_teleop`**:
    *   Scripts for teleoperating the robot using a keyboard or joystick.

### Warehouse Application Packages (`warehouse/`)

*   **`warehouse_rover_mission_control`**:
    *   High-level mission executive that coordinates autonomous scanning tasks.
    *   Manages the state machine for navigating to racks and performing inventory checks.
*   **`warehouse_rover_qr_detection`**:
    *   Detects and parses QR codes from camera feeds using OpenCV and ZBar.
*   **`warehouse_rover_image_processing`**:
    *   Performs image preprocessing to improve QR detection rates.
*   **`warehouse_rover_rack_detection`**:
    *   Analyzes occupancy grid maps to identify rack locations dynamically.
*   **`warehouse_rover_lift_control`**:
    *   Controls the vertical lift mechanism for scanning shelves at different heights.
*   **`warehouse_rover_database`**:
    *   Manages a SQLite database to store inventory data scanned by the robot.
*   **`warehouse_rover_msgs`**:
    *   Defines custom ROS 2 messages and services used across the warehouse packages.

## Installation

1.  **Clone the repository:**
    ```bash
    cd ~/ros_ws/src
    # Clone your repo here
    ```

2.  **Install dependencies:**
    ```bash
    cd ~/ros_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
    *Note: Ensure you have `libopencv-dev`, `libzbar-dev`, and `libsqlite3-dev` installed.*

3.  **Build the workspace:**
    ```bash
    colcon build
    ```

4.  **Source the setup script:**
    ```bash
    source install/setup.bash
    ```

## Usage

### 1. Simulation
Launch the robot in the Gazebo warehouse environment:
```bash
ros2 launch mecanum_gazebo simulation_world.launch.py
```

### 2. Navigation
Launch the navigation stack (AMCL + Nav2):
```bash
ros2 launch mecanum_navigation_setup cartographer_navigation.launch.py
```
*Or for SLAM (Mapping):*
```bash
ros2 launch mecanum_navigation_setup cartographer.launch.py
```

### 3. Warehouse Mission
Run the mission control node to start an inventory scan:
```bash
ros2 launch warehouse_rover_mission_control master_autonomous_warehouse.launch.py
```

## Dependencies

*   **ROS 2** (Humble/Iron/Rolling)
*   **Nav2** (Navigation 2 Stack)
*   **Gazebo** (Simulation)
*   **OpenCV** (Computer Vision)
*   **ZBar** (QR Code Reading)
*   **SQLite3** (Database)
