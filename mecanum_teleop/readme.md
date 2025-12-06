# mecanum_teleop

Keyboard teleoperation package for controlling the mecanum drive robot.

## Overview

Provides a keyboard interface to control the robot's holonomic movement by publishing `TwistStamped` messages to `/mecanum_drive_controller/reference`.

## Usage

```bash
ros2 run mecanum_teleop mecanum_teleop_key
```

## Controls

**Movement:**
- `w/s` - Forward/Backward
- `a/d` - Strafe Left/Right
- `q/e` - Diagonal Forward-Left/Right
- `z/c` - Diagonal Backward-Left/Right
- `j/l` - Rotate Left/Right
- `k` or `SPACE` - Stop

**Speed:**
- `+/-` - Increase/Decrease speed
- `r` - Reset to default speed
- `i` - Print current status

Press `Ctrl+C` to exit.
