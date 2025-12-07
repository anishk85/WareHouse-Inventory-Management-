#!/bin/bash
# Quick script to start mapping mode

echo "=========================================="
echo "  Starting MAPPING mode"
echo "=========================================="

cd /root/ros2_ws
source install/setup.bash

ros2 launch mecanum_hardware complete_mapping_navigation.launch.py mode:=mapping
