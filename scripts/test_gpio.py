#!/usr/bin/env python3
"""
MD10S Motor Test using pigpio
Works on all Raspberry Pi models including Pi 5
"""

import pigpio
import time

# Pin configuration (BCM numbering)
PWM_PIN = 18
DIR_PIN = 23

print("=" * 60)
print("MD10S Motor Driver Test (using pigpio)")
print("=" * 60)

# Connect to pigpio daemon
pi = pigpio.pi()

if not pi.connected:
    print("ERROR: Cannot connect to pigpio daemon")
    print("Run: sudo pigpiod")
    exit(1)

# Setup pins
pi.set_mode(PWM_PIN, pigpio.OUTPUT)
pi.set_mode(DIR_PIN, pigpio.OUTPUT)

# Set PWM frequency to 1000 Hz
pi.set_PWM_frequency(PWM_PIN, 1000)

print("GPIO Setup Complete\n")

try:
    # Test 1: Full speed forward
    print("TEST 1: FULL SPEED FORWARD (5 sec)")
    pi.write(DIR_PIN, 1)  # Direction HIGH
    pi.set_PWM_dutycycle(PWM_PIN, 255)  # 100% (0-255 range)
    time.sleep(5)
    pi.set_PWM_dutycycle(PWM_PIN, 0)
    print("  → Motor should have moved FORWARD at full speed\n")
    time.sleep(2)
    
    # Test 2: Full speed reverse
    print("TEST 2: FULL SPEED REVERSE (5 sec)")
    pi.write(DIR_PIN, 0)  # Direction LOW
    pi.set_PWM_dutycycle(PWM_PIN, 255)  # 100%
    time.sleep(5)
    pi.set_PWM_dutycycle(PWM_PIN, 0)
    print("  → Motor should have moved REVERSE at full speed\n")
    time.sleep(2)
    
    # Test 3: Half speed forward
    print("TEST 3: HALF SPEED FORWARD (5 sec)")
    pi.write(DIR_PIN, 1)
    pi.set_PWM_dutycycle(PWM_PIN, 128)  # 50% (128/255)
    time.sleep(5)
    pi.set_PWM_dutycycle(PWM_PIN, 0)
    print("  → Motor should have moved FORWARD at half speed\n")
    time.sleep(2)
    
    # Test 4: Quarter speed
    print("TEST 4: QUARTER SPEED FORWARD (5 sec)")
    pi.write(DIR_PIN, 1)
    pi.set_PWM_dutycycle(PWM_PIN, 64)  # 25% (64/255)
    time.sleep(5)
    pi.set_PWM_dutycycle(PWM_PIN, 0)
    print("  → Motor should have moved FORWARD slowly\n")
    
    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    print("\nDid the motor move in ANY test?")
    print("  YES → Hardware is working! Issue is in Docker/ROS setup")
    print("  NO  → Check these in order:")
    print("        1. Motor power supply is ON (12V/24V)")
    print("        2. Common ground connected:")
    print("           RPi GND ← wire → Power Supply GND")
    print("        3. Wiring:")
    print("           MD10S PWM ← GPIO18 (Physical pin 12)")
    print("           MD10S DIR ← GPIO23 (Physical pin 16)")
    print("           MD10S GND ← RPi GND")
    print("        4. Motor wires connected to MD10S M+ and M-")
    print("        5. Use multimeter: GPIO18 should show ~3.3V during test")
    
except KeyboardInterrupt:
    print("\n\nStopped by user")

finally:
    # Cleanup
    pi.set_PWM_dutycycle(PWM_PIN, 0)
    pi.write(DIR_PIN, 0)
    pi.stop()
    print("\nGPIO Cleaned up")