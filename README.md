

This repository contains a full-stack ROS 2 (Humble) project to simulate a mecanum-wheeled robot in Gazebo, run teleop, publish odometry & TF, and experiment with mapping and navigation (Nav2, SLAM Toolbox, Cartographer). Some SLAM configurations are present but require tuning — see "Known issues".

---

##  Prerequisites (tested)

* **OS**: Ubuntu 22.04 (Jammy)
* **ROS 2**: Humble Hawksbill
* **Python**: 3.10+
* **Gazebo**: Classic 11
* **Build tool**: colcon

### Recommended system packages

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  git \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-robot-localization
```

> Tip: If you use a minimal ROS install, make sure `ros-humble-desktop` (or at minimum `ros-humble-robot-state-publisher`, `ros-humble-xacro`, and `ros-humble-gazebo-ros-pkgs`) are installed.

---

## Installation

1. Create the workspace (if you don't have one):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

2. Clone the repo:

```bash
git clone https://github.com/anishk85/Inter-IIT-Robotics-Midprep.git 
cd ~/ros2_ws
```

3. Install package dependencies (rosdep):

```bash
sudo rosdep init  # only if rosdep not initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

> If you use `zsh` or `fish`, source the appropriate setup file (`install/setup.zsh` etc.).

---

## Project structure (short)

```
Inter-IIT-Robotics-Midprep/
├── mecanum_in_gazebo/          # Robot description, Gazebo world, controllers, scripts
│   ├── arena/                  # Gazebo world + models
│   ├── config/                 # controller.yaml, other configs
│   ├── launch/                 # launch files (gazebo, spawner, controllers)
│   ├── scripts/                # teleop, cmd_vel->mecanum, odom TF publisher
│   └── urdf/                   # xacro URDFs (robot + sensors)
└── navigation_setup/           # Nav2, SLAM & sensor-fusion config + launch files
```

---

## Quick start — run the simulation

**1) Launch the complete simulation in Gazebo:**

```bash
ros2 launch mecanum_in_gazebo final_gazebo.launch.py
```

This starts Gazebo, spawns the robot with sensors, and starts controllers (if configured).

**2) Teleoperation (keyboard):**

```bash
ros2 run mecanum_in_gazebo final_teleop.py
```

**Keyboard controls**:

* `w` — forward
* `s` — backward
* `a` / `d` — strafe left / right
* `q` / `e` — rotate CCW / CW
* `x` — stop
* `+` / `-` — change speed


This will run SLAM Toolbox in online mode and produce a map that you can save with the `map_server`.

### Cartographer (experimental)

Cartographer configs are present (`mecanum_cartographer_2d.lua`) but require tuning of sensor frame names, transforms, and extrinsics. Use `cartographer_ros` only if you understand Cartographer's config parameters.

> **Known**: SLAM setup in this repo is *not* fully tuned. Expect to tune `sensor_topic` names, frame IDs, and the robot's odometry parameters.

---


##  Controllers & important config

* `config/controller.yaml` contains the ros2_control controller definitions used by the Gazebo robot when the `ros2_control` plugin is enabled.
* If controllers aren't loading: ensure `controller_manager` is running and that joint names in the URDF match the controllers' `joints:` list.

**Common pitfall**: mismatched joint names in URDF vs `controller.yaml` cause controllers not to load. Double-check `mec_rob.xacro` joint names.

---

##  Development notes

* The `scripts/` folder includes a working `final_teleop.py`. Some other scripts are prototypes — review before using in production.


---

