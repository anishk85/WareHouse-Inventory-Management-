# Warehouse Rover Lift Control

Service-based lift controller for automated shelf scanning.

## Services

- `/lift_controller/scan_cycle` - Full scan cycle (all shelves)
- `/lift_controller/reset` - Return to home position

## Configuration

Edit `config/lift_controller_params.yaml`:
- `shelf_heights`: Array of shelf positions (meters)
- `move_duration`: Time per movement (seconds)
- `dwell_time`: Scan time per shelf (seconds)

## Usage
```bash
# Launch
ros2 launch warehouse_rover_lift_control lift_controller.launch.py

# Test scan
ros2 service call /lift_controller/scan_cycle std_srvs/srv/Trigger
```
