# Warehouse Rover Application Suite

Autonomous warehouse inventory management system for mecanum-wheeled robots using ROS 2.

## Overview

This package provides high-level autonomous warehouse operations including QR code scanning, rack detection, lift control, and database management for inventory tracking.

## Packages

### `warehouse_rover_mission_control`
Mission executive that coordinates autonomous scanning tasks and manages state machine for navigation and inventory checks.

### `warehouse_rover_qr_detection`
QR code detection and parsing using OpenCV and ZBar from camera feeds.

### `warehouse_rover_image_processing`
Image preprocessing pipeline to improve QR detection accuracy.

### `warehouse_rover_rack_detection`
Dynamic rack location identification from occupancy grid maps.

### `warehouse_rover_database`
SQLite database management for storing scanned inventory data.

### `warehouse_rover_msgs`
Custom ROS 2 message and service definitions for warehouse operations.

## Quick Start


### Launch Mission Control


```bash
ros2 launch mecanum_gazebo simualtion_world.launch.py
```


```bash
ros2 launch warehouse_rover_mission_control mission.launch.py
```
