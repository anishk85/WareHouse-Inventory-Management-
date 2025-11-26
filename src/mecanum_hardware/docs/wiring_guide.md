# Mecanum Robot Wiring Guide

## Overview
- 4 NEMA23 Stepper Motors (one per wheel)
- 4 Stepper Drivers (DM542T, TB6600, or similar)
- 2 ESP32 Controllers
- 1 Raspberry Pi 5 (16GB)
- 1 LiDAR sensor
- 1 Depth Camera (RealSense D435/D455 or similar)
- 1 IMU sensor

## Motor to ESP Mapping

```
ESP1 (FRONT) - /dev/ttyUSB0
├─ Motor 1 → Front Left Wheel (FL)
└─ Motor 2 → Front Right Wheel (FR)

ESP2 (BACK) - /dev/ttyUSB1
├─ Motor 1 → Back Left Wheel (BL)
└─ Motor 2 → Back Right Wheel (BR)
```

## ESP32 Pin Assignments

### ESP1 (Front Motors)
```
Motor 1 (Front Left):
- GPIO 12 → Driver STEP/PUL
- GPIO 14 → Driver DIR
- GPIO 27 → Driver EN (Enable)

Motor 2 (Front Right):
- GPIO 26 → Driver STEP/PUL
- GPIO 25 → Driver DIR
- GPIO 33 → Driver EN (Enable)
```

### ESP2 (Back Motors)
```
Motor 1 (Back Left):
- GPIO 12 → Driver STEP/PUL
- GPIO 14 → Driver DIR
- GPIO 27 → Driver EN (Enable)

Motor 2 (Back Right):
- GPIO 26 → Driver STEP/PUL
- GPIO 25 → Driver DIR
- GPIO 33 → Driver EN (Enable)
```

## Stepper Driver Configuration

### DIP Switch Settings (example for 1/1 microstepping)
```
DIP1: OFF  DIP2: OFF  DIP3: OFF
Microstep Resolution: Full step (200 steps/rev)
```

Adjust `MICROSTEPS` in firmware accordingly:
- 1/1:  MICROSTEPS = 1
- 1/2:  MICROSTEPS = 2
- 1/4:  MICROSTEPS = 4
- 1/8:  MICROSTEPS = 8
- 1/16: MICROSTEPS = 16

### Current Setting
Set driver current to 80% of motor rated current using potentiometer

## Power Connections

```
24V Power Supply (5A or higher)
├─ Stepper Driver 1 (VCC, GND)
├─ Stepper Driver 2 (VCC, GND)
├─ Stepper Driver 3 (VCC, GND)
└─ Stepper Driver 4 (VCC, GND)

5V Power Supply (3A)
├─ ESP32 #1 (5V, GND via USB or VIN)
└─ ESP32 #2 (5V, GND via USB or VIN)

RPi 5 - USB-C Power (5V 5A)
```

## USB Connection Topology

```
Raspberry Pi 5
├─ USB0 → ESP32 #1 (Front motors)
├─ USB1 → ESP32 #2 (Back motors)
├─ USB2 → LiDAR
├─ USB3 → Depth Camera (RealSense)
└─ USB4 → IMU (or I2C via GPIO)
```

## Identifying USB Ports

```bash
# Before connecting devices
ls /dev/ttyUSB*

# Connect ESP1, check new port
# Connect ESP2, check new port
# Connect LiDAR, check new port

# Create udev rules for persistent naming
sudo nano /etc/udev/rules.d/99-mecanum-robot.rules
```

Add:
```
# ESP32 Controllers
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp_front"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp_back"

# LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar"
```

Reload udev:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Safety Considerations

1. **Emergency Stop**: Always have physical E-STOP button
2. **Current Limiting**: Set driver current correctly
3. **Cooling**: Ensure adequate cooling for drivers
4. **Wiring**: Use appropriate gauge wire for motor power
5. **Grounding**: Common ground for all components