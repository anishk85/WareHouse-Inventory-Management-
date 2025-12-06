
# Inter-IIT Warehouse Robot Workspace

This ROS 2 workspace contains the comprehensive software stack for a mecanum-wheeled mobile robot designed for autonomous warehouse inventory management. The system integrates simulation environments, hardware interfacing, autonomous navigation, and specialized applications for QR code scanning and rack detection.

## Workspace Structure

The workspace architecture is divided into **Core Robot Packages** (hardware and navigation) and **Warehouse Application Packages** (high-level logic and sensing).

### 1\. Core Robot Packages

These packages handle the low-level control, simulation, and navigation capabilities.

  * **`mecanum_hardware`** (via `mecanum_bringup`)
      * Provides the hardware interface for controlling stepper motors and reading sensors.
      * Manages serial communication with the ESP32 firmware.
  * **`mecanum_description`**
      * Contains URDF and Xacro robot descriptions.
      * Defines physical properties, visual meshes, and collision models.
  * **`mecanum_gazebo`**
      * Simulation environment powered by Gazebo.
      * Includes world files and launch configurations to spawn the robot in a simulated warehouse.
  * **`mecanum_navigation_setup`**
      * Configuration and launch files for the ROS 2 Navigation Stack (Nav2).
      * Integrates SLAM Toolbox and Cartographer for mapping and real-time localization.
  * **`mecanum_teleop`**
      * Scripts for manual robot control via keyboard or joystick.

### 2\. Warehouse Application Packages (`warehouse/`)

These packages implement specific mission logic for the Inter-IIT challenge.

  * **`warehouse_rover_mission_control`**
      * **High-Level Executive:** Coordinates autonomous scanning tasks.
      * **State Machine:** Manages navigation to racks and inventory check sequences.
  * **`warehouse_rover_qr_detection`**
      * Detects and parses QR codes from video feeds utilizing OpenCV and ZBar.
  * **`warehouse_rover_image_processing`**
      * Performs pre-processing on raw image data to maximize QR detection accuracy.
  * **`warehouse_rover_rack_detection`**
      * Analyzes occupancy grid maps to dynamically identify rack locations.
  * **`warehouse_rover_lift_control`**
      * Controls the vertical lift mechanism to facilitate scanning at various shelf heights.
  * **`warehouse_rover_database`**
      * Manages a SQLite database to persistently store scanned inventory data.
  * **`warehouse_rover_msgs`**
      * Defines custom ROS 2 messages and services used for inter-node communication.

-----

##  Installation

The environment is containerized for ease of use. Follow these steps to set up the Docker container.

**1. Pull the Docker Image**

```bash
docker pull fibonacci69/inter_iit_workspace_gui:latest
```

**2. Grant GUI Access**
Allow the Docker container to access the host display (X11).

```bash
xhost +local:docker
```

**3. Launch the Container**
Run the container with host networking and display forwarding enabled.

```bash
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  fibonacci69/inter_iit_workspace_gui:latest
```

4. Install Package Dependencies (rosdep)

Once inside the container and in the workspace source directory, use rosdep to install all required system dependencies (libraries, packages not included in the container image).

```bash



# This assumes you are in the root of the workspace directory

rosdep install -i --from-path src --rosdistro humble -y
```

5. Build the Workspace (colcon)

Use colcon to compile all the packages in the workspace.

```Bash



# This builds the packages using parallel jobs (recommended)

colcon build --symlink-install

```

-----

## Usage

Once inside the container, you can execute the following commands to run specific modules.

### 1\. Simulation

Spawns the robot in the Gazebo warehouse environment.

```bash
ros2 launch mecanum_gazebo simulation_world.launch.py
```

### 2\. Navigation

Launch the navigation stack using your preferred method.

**Option A: Navigation (AMCL + Nav2)**
*Use this for navigating a known map.*

```bash
ros2 launch mecanum_navigation_setup cartographer_navigation.launch.py
```

**Option B: SLAM (Mapping)**
*Use this to generate a new map of the environment.*

```bash
ros2 launch mecanum_navigation_setup slam.launch.py
```

### 3\. Warehouse Mission

Initiates the master node to begin the autonomous inventory scan.

```bash
ros2 launch warehouse_rover_mission_control master_autonomous_warehouse.launch.py
```

## Demos and Results

<div align="center">

<h3>1. Real Hardware Mapping</h3>
<img src="images/real_hardware_mapping.gif" width="70%" alt="Hardware Mapping"/>

<h3>2. Hardware Navigation</h3>
<img src="images/navigation.gif" width="70%" alt="Hardware Navigation"/>

<h3>3. Z-Axis Movement</h3>
<img src="images/z-axis_hardware.gif" width="70%" alt="Z-Axis Movement"/>

<h3>4. Obstacle Avoidance</h3>
<img src="images/obstacle_avoid.gif" width="70%" alt="Obstacle Avoidance"/>

</div>





-----
