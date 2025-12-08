#!/bin/bash

# Warehouse Rover Web GUI Quick Start Script

echo "=========================================="
echo "  Warehouse Rover Web GUI Launcher"
echo "=========================================="
echo ""

# Check if in correct directory
if [ ! -d "/root/ros2_ws" ]; then
    echo "❌ Error: /root/ros2_ws directory not found"
    exit 1
fi

cd /root/ros2_ws

# Source ROS2 workspace
echo "📦 Sourcing ROS2 workspace..."
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
echo "✅ Workspace sourced"
echo ""

# Get network IP
NETWORK_IP=$(hostname -I | awk '{print $1}')
echo "🌐 Network Information:"
echo "   Docker IP: $NETWORK_IP"
echo "   WebSocket: ws://$NETWORK_IP:9090"
echo "   Web GUI:   http://$NETWORK_IP:3000"
echo ""

echo "=========================================="
echo "  Starting Services"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down services..."
    kill $BRIDGE_PID $NEXTJS_PID 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start ROS2 WebSocket Bridge
echo "1️⃣  Starting ROS2 WebSocket Bridge..."
ros2 run mecanum_hardware ros2_web_bridge.py &
BRIDGE_PID=$!
echo "   PID: $BRIDGE_PID"
sleep 3

# Start Next.js Dev Server
echo ""
echo "2️⃣  Starting Next.js Web GUI..."
cd /root/ros2_ws/my-app/my-app
npm run dev &
NEXTJS_PID=$!
echo "   PID: $NEXTJS_PID"
sleep 5

echo ""
echo "=========================================="
echo "  ✅ All Services Running!"
echo "=========================================="
echo ""
echo "📱 Access from your laptop:"
echo "   http://$NETWORK_IP:3000"
echo ""
echo "🔧 Launch Control Page:"
echo "   http://$NETWORK_IP:3000/launch"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Wait for processes
wait
