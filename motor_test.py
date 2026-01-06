#!/usr/bin/env python3
"""
Simple MD10S Motor Driver Test Script
Run directly on Raspberry Pi (outside Docker)
"""

import gpiod
import time

# Pin configuration (BCM numbering)
PWM_PIN = 18
DIR_PIN = 23

print("=" * 60)
print("MD10S Motor Driver Simple Test")
print("=" * 60)
print(f"PWM Pin: GPIO{PWM_PIN}")
print(f"DIR Pin: GPIO{DIR_PIN}")
print()

# Check connections
print("CONNECTIONS CHECK:")
print("  MD10S PWM  → Raspberry Pi GPIO18 (Pin 12)")
print("  MD10S DIR  → Raspberry Pi GPIO23 (Pin 16)")
print("  MD10S GND  → Raspberry Pi GND")
print("  MD10S VIN  → Motor Power Supply (+)")
print("  Power GND  → Motor Power Supply (-) AND RPi GND (common ground!)")
print("  MD10S M+   → Motor positive")
print("  MD10S M-   → Motor negative")
print()

try:
    # Initialize GPIO
    chip = gpiod.Chip('gpiochip0')
    pwm_line = chip.get_line(PWM_PIN)
    dir_line = chip.get_line(DIR_PIN)
    
    pwm_line.request(consumer="motor_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    dir_line.request(consumer="motor_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    
    print("✓ GPIO initialized\n")
    
    # Test 1: Full speed forward
    print("TEST 1: Full Speed Forward (5 seconds)")
    print("  DIR = HIGH, PWM = HIGH")
    print("  Motor should run at FULL SPEED")
    dir_line.set_value(1)  # Forward direction
    pwm_line.set_value(1)  # Full speed (100% duty cycle)
    time.sleep(5)
    pwm_line.set_value(0)
    print("  → Did the motor move? [Expected: YES at full speed]\n")
    time.sleep(2)
    
    # Test 2: Full speed reverse
    print("TEST 2: Full Speed Reverse (5 seconds)")
    print("  DIR = LOW, PWM = HIGH")
    print("  Motor should run REVERSE at FULL SPEED")
    dir_line.set_value(0)  # Reverse direction
    pwm_line.set_value(1)  # Full speed
    time.sleep(5)
    pwm_line.set_value(0)
    print("  → Did the motor move? [Expected: YES in reverse]\n")
    time.sleep(2)
    
    # Test 3: Half speed with software PWM
    print("TEST 3: Half Speed Forward (5 seconds)")
    print("  DIR = HIGH, PWM = 50% duty cycle")
    print("  Motor should run at HALF SPEED")
    dir_line.set_value(1)
    
    start = time.time()
    while time.time() - start < 5:
        pwm_line.set_value(1)
        time.sleep(0.0005)  # 0.5ms on
        pwm_line.set_value(0)
        time.sleep(0.0005)  # 0.5ms off = 1kHz, 50% duty
    
    print("  → Did the motor move? [Expected: YES at half speed]\n")
    time.sleep(2)
    
    # Test 4: Slow speed (25% duty)
    print("TEST 4: Slow Speed Forward (5 seconds)")
    print("  DIR = HIGH, PWM = 25% duty cycle")
    print("  Motor should run SLOWLY")
    dir_line.set_value(1)
    
    start = time.time()
    while time.time() - start < 5:
        pwm_line.set_value(1)
        time.sleep(0.00025)  # 0.25ms on
        pwm_line.set_value(0)
        time.sleep(0.00075)  # 0.75ms off = 1kHz, 25% duty
    
    print("  → Did the motor move? [Expected: YES slowly]\n")
    time.sleep(2)
    
    # Stop
    print("Stopping motor...")
    pwm_line.set_value(0)
    dir_line.set_value(0)
    
    # Cleanup
    pwm_line.release()
    dir_line.release()
    chip.close()
    
    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    print("\nRESULTS ANALYSIS:")
    print("  ✓ If motor moved in Test 1: Hardware is working!")
    print("  ✗ If motor NEVER moved in any test:")
    print("      1. Check motor power supply is ON")
    print("      2. Verify common ground between RPi and power supply")
    print("      3. Test with multimeter:")
    print("         - GPIO18 should show 3.3V when PWM is HIGH")
    print("         - GPIO23 should toggle between 0V and 3.3V")
    print("      4. Check MD10S power LED (if present)")
    print("      5. Try swapping M+ and M- wires")
    print("      6. Test motor directly with power supply")
    
except KeyboardInterrupt:
    print("\n\nTest interrupted by user")
    pwm_line.set_value(0)
    dir_line.set_value(0)
    pwm_line.release()
    dir_line.release()
    chip.close()
    
except Exception as e:
    print(f"\n✗ ERROR: {e}")
    print("\nTroubleshooting:")
    print("  - Make sure you have python3-libgpiod installed")
    print("  - Run with: sudo python3 script.py")
    print("  - Check /dev/gpiochip0 exists")
