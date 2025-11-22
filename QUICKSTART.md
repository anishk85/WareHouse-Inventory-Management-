# 🚀 Quick Start Guide

Get the warehouse robot running in **5 minutes**!

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble installed
- ~10GB free disk space

## Installation
```bash
# 1. Create workspace
mkdir -p ~/interiit/src && cd ~/interiit/src

# 2. Clone repository
git clone <your-repo-url> .

# 3. Install dependencies
cd ~/interiit
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install

# 5. Source
echo "source ~/interiit/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Run Demo

### Option 1: Quick Demo (Simulation Only)
```bash
ros2 launch mecanum_in_gazebo gazebo.launch.py
```

### Option 2: Full Autonomous Mission
```bash
ros2 launch warehouse_rover_mission_control master_autonomous_warehouse.launch.py
```

Watch the robot automatically:
1. Navigate to 5 racks
2. Scan QR codes at 4 heights each
3. Save inventory to database

## Troubleshooting

**Robot doesn't move?**
```bash
ros2 topic echo /cmd_vel  # Check velocity commands
ros2 topic list | grep mecanum  # Verify topics
```

**Nav2 not starting?**
```bash
ros2 lifecycle list  # Check node states
```

**QR codes not detected?**
```bash
ros2 run rqt_image_view rqt_image_view  # View camera feed
```

## Next Steps

- Read full [README.md](README.md)
- Check [ARCHITECTURE.md](ARCHITECTURE.md) for system design
- See [CONTRIBUTING.md](CONTRIBUTING.md) to contribute

---

**Need help?** Open an [issue](https://github.com/YourOrg/Inter-IIT-Robotics-Midprep/issues)
