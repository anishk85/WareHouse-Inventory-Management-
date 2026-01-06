#!/bin/bash
# ROS2 Humble Container Launch Script for Raspberry Pi
# Features: Port forwarding, GUI support, persistent workspace, USB camera support

set -e

# ============================================================================
# CONFIGURATION
# ============================================================================

CONTAINER_NAME="ros2_humble_rpi"
LOCAL_WS_DIR="$HOME/ros2_ws"
ROS2_IMAGE="arm64v8/ros:humble-ros-base"  # ARM64 compatible image

# Network Port forwarding (HOST:CONTAINER)
NETWORK_PORTS=(
    "11311:11311"   # ROS Master (if using ros1_bridge)
    "9090:9090"     # Rosbridge WebSocket
    "8080:8080"     # Web interfaces
    "5900:5900"     # VNC (if needed)
)

# Git repo
REPO_URL="https://github.com/anishk85/Inter-IIT-Robotics-Midprep.git"
REPO_BRANCH="shifting-to-mecanum"

# ============================================================================
# SETUP X AUTHORITY FOR GUI
# ============================================================================

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔑 Setting up X authority for GUI forwarding..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

export XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - 2>/dev/null || true
chmod 644 $XAUTH

echo "🖥️  Allowing local Docker connections to X server..."
xhost +local:docker > /dev/null 2>&1 || echo "⚠️  X server not available (headless mode OK)"

# ============================================================================
# DETECT USB CAMERAS
# ============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📹 Detecting USB cameras..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

VIDEO_DEVICES=""
for dev in /dev/video*; do
    if [ -c "$dev" ]; then
        echo "   ✓ Found: $dev"
        VIDEO_DEVICES="$VIDEO_DEVICES --device=$dev"
    fi
done

if [ -z "$VIDEO_DEVICES" ]; then
    echo "   ⚠️  No video devices detected"
else
    echo "   ✅ Video devices will be forwarded to container"
fi

# ============================================================================
# CREATE LOCAL WORKSPACE
# ============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📁 Setting up local workspace..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

mkdir -p $LOCAL_WS_DIR/src
echo "✅ Workspace created at: $LOCAL_WS_DIR"

# ============================================================================
# BUILD PORT MAPPING ARGUMENTS
# ============================================================================

PORT_ARGS=""
for port in "${NETWORK_PORTS[@]}"; do
    PORT_ARGS="$PORT_ARGS -p $port"
done

echo ""
echo "🔌 Port forwarding configured:"
for port in "${NETWORK_PORTS[@]}"; do
    echo "   → $port"
done

# ============================================================================
# CONTAINER MANAGEMENT
# ============================================================================

if [ "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "▶️  Found existing container. Restarting..."
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    docker stop ${CONTAINER_NAME} > /dev/null 2>&1 || true
    docker start ${CONTAINER_NAME} > /dev/null
    
    echo "✅ Container started! Entering interactive shell..."
    docker exec -it --workdir /root/ros2_ws ${CONTAINER_NAME} /bin/bash

else
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🚀 Creating new container with full setup..."
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    docker run -it \
      --name ${CONTAINER_NAME} \
      --hostname ros2-rpi \
      --workdir="/root/ros2_ws" \
      --privileged \
      --add-host=host.docker.internal:127.0.0.1 \
      --net=host \
      $PORT_ARGS \
      --env="DISPLAY=$DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --env="XAUTHORITY=$XAUTH" \
      --env="ROS_DOMAIN_ID=0" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --volume="$XAUTH:$XAUTH" \
      --volume="$LOCAL_WS_DIR:/root/ros2_ws" \
      --volume="/dev:/dev" \
      --device-cgroup-rule='a *:* rmw' \
      --restart unless-stopped \
      --device=/dev/input/js0 \
      --device=/dev/i2c-1 \
      $VIDEO_DEVICES \
      ${ROS2_IMAGE} \
      /bin/bash -c "
        set -e
        
        echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        echo '🛠️  Performing first-time container setup...'
        echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        
        # Update package lists
        apt-get update -qq
        
        # Install essential tools + camera/video tools
        apt-get install -y -qq \
          git wget curl vim nano \
          python3-pip python3-colcon-common-extensions \
          python3-rosdep python3-vcstool \
          build-essential cmake \
          net-tools iputils-ping iproute2 \
          can-utils i2c-tools \
          v4l-utils ffmpeg \
          libv4l-dev \
          ros-humble-cv-bridge \
          ros-humble-image-transport \
          ros-humble-camera-info-manager \
          ros-humble-usb-cam > /dev/null
        
        # Initialize rosdep
        if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
          rosdep init > /dev/null 2>&1 || true
        fi
        rosdep update > /dev/null 2>&1 || true
        
        # Setup ROS2 env
        if ! grep -q 'source /opt/ros/humble/setup.bash' /root/.bashrc; then
          echo '' >> /root/.bashrc
          echo '# ROS2 Humble setup' >> /root/.bashrc
          echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
          echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc
          echo 'export ROS_LOCALHOST_ONLY=0' >> /root/.bashrc
          echo '' >> /root/.bashrc
          echo '# Auto-source workspace' >> /root/.bashrc
          echo 'if [ -f /root/ros2_ws/install/setup.bash ]; then' >> /root/.bashrc
          echo '  source /root/ros2_ws/install/setup.bash' >> /root/.bashrc
          echo 'fi' >> /root/.bashrc
          echo '' >> /root/.bashrc
          echo '# Aliases' >> /root/.bashrc
          echo 'alias cb=\"cd /root/ros2_ws && colcon build --symlink-install\"' >> /root/.bashrc
          echo 'alias cs=\"source /root/ros2_ws/install/setup.bash\"' >> /root/.bashrc
          echo 'alias cam-list=\"v4l2-ctl --list-devices\"' >> /root/.bashrc
          echo 'alias cam-info=\"v4l2-ctl --device=/dev/video0 --list-formats-ext\"' >> /root/.bashrc
        fi
        
        # Clone repository
        if [ ! -z '$REPO_URL' ]; then
          cd /root/ros2_ws/src
          if [ ! -d .git ]; then
            git clone -b $REPO_BRANCH $REPO_URL .
          fi
        fi
        
        # Create example package if empty
        if [ -z \"\$(ls -A /root/ros2_ws/src)\" ]; then
          cd /root/ros2_ws/src
          ros2 pkg create --build-type ament_cmake sample_package
        fi
        
        echo ''
        echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        echo '✅ Initial setup complete!'
        echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        echo ''
        echo '📹 Camera Testing Commands:'
        echo '   cam-list              - List all cameras'
        echo '   cam-info              - Show camera formats'
        echo '   ffplay /dev/video0    - Test camera feed'
        echo ''
        echo '🎥 ROS2 USB Camera Node:'
        echo '   ros2 run usb_cam usb_cam_node_exe'
        echo ''
        
        exec /bin/bash
      "
fi

# ============================================================================
# CLEANUP
# ============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🧹 Cleaning up X server permissions..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
xhost -local:docker > /dev/null 2>&1 || true

echo "✅ Done! Container exited cleanly."
echo ""