#!/bin/bash
# Quick script to start navigation mode

echo "=========================================="
echo "  Starting NAVIGATION mode"
echo "=========================================="

if [ -z "$1" ]; then
    echo "Usage: ./start_navigation.sh <path_to_map.yaml>"
    echo "Example: ./start_navigation.sh ~/maps/my_map.yaml"
    exit 1
fi

cd /root/ros2_ws
source install/setup.bash

ros2 launch mecanum_hardware complete_mapping_navigation.launch.py \
    mode:=navigation \
    map:=$1
