#!/bin/bash
RPI_IP="192.168.1.100"  # Change to your RPi IP
RPI_USER="ubuntu"        # Change to your RPi username

echo "Deploying to RPi at $RPI_IP..."

# Create workspace on RPi
ssh $RPI_USER@$RPI_IP "mkdir -p ~/ros2_ws/src"

# Copy packages
rsync -avz --exclude build --exclude install --exclude log \
  ~/002/src/mecanum_hardware \
  ~/002/src/mecanum_in_gazebo \
  ~/002/src/navigation_setup \
  $RPI_USER@$RPI_IP:~/ros2_ws/src/

# Build on RPi
ssh $RPI_USER@$RPI_IP << 'EOF'
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mecanum_hardware mecanum_in_gazebo navigation_setup
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
EOF

echo "Deployment complete!"