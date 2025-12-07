#!/usr/bin/env python3
"""
GPIO Hardware Test Script for MD10S Motor Driver
Tests PWM and DIR pins to verify hardware connections
"""

import gpiod
import time
import sys

# Pin configuration
PWM_PIN = 18
DIR_PIN = 23

def test_gpio():
    print("=" * 60)
    print("GPIO Hardware Test for MD10S Motor Driver")
    print("=" * 60)
    print(f"PWM Pin: GPIO{PWM_PIN}")
    print(f"DIR Pin: GPIO{DIR_PIN}")
    print()
    
    try:
        # Initialize GPIO
        chip = gpiod.Chip('gpiochip0')
        pwm_line = chip.get_line(PWM_PIN)
        dir_line = chip.get_line(DIR_PIN)
        
        pwm_line.request(consumer="test_pwm", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        dir_line.request(consumer="test_dir", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        
        print("✓ GPIO initialized successfully")
        print()
        
        # Test 1: DIR pin HIGH and LOW
        print("TEST 1: Direction Pin Toggle")
        print("-" * 40)
        for i in range(3):
            print(f"  Setting DIR to HIGH (Round {i+1})")
            dir_line.set_value(1)
            time.sleep(1)
            print(f"  Setting DIR to LOW (Round {i+1})")
            dir_line.set_value(0)
            time.sleep(1)
        print("✓ Direction pin test complete\n")
        
        # Test 2: PWM pin solid HIGH
        print("TEST 2: PWM Pin Solid HIGH (5 seconds)")
        print("-" * 40)
        print("  Motor should run at full speed")
        pwm_line.set_value(1)
        dir_line.set_value(1)  # Set direction UP
        time.sleep(5)
        pwm_line.set_value(0)
        print("✓ PWM solid HIGH test complete\n")
        
        # Test 3: Software PWM at 50% duty cycle
        print("TEST 3: Software PWM at 50% Duty Cycle (5 seconds)")
        print("-" * 40)
        print("  Motor should run at half speed")
        dir_line.set_value(1)
        
        start_time = time.time()
        while time.time() - start_time < 5:
            pwm_line.set_value(1)
            time.sleep(0.0005)  # 0.5ms ON
            pwm_line.set_value(0)
            time.sleep(0.0005)  # 0.5ms OFF (1kHz, 50% duty)
        
        print("✓ Software PWM test complete\n")
        
        # Test 4: Reverse direction
        print("TEST 4: Reverse Direction (5 seconds)")
        print("-" * 40)
        print("  Motor should run in opposite direction")
        dir_line.set_value(0)  # Reverse direction
        
        start_time = time.time()
        while time.time() - start_time < 5:
            pwm_line.set_value(1)
            time.sleep(0.0005)
            pwm_line.set_value(0)
            time.sleep(0.0005)
        
        print("✓ Reverse direction test complete\n")
        
        # Cleanup
        print("Cleaning up...")
        pwm_line.set_value(0)
        dir_line.set_value(0)
        pwm_line.release()
        dir_line.release()
        chip.close()
        
        print("\n" + "=" * 60)
        print("All tests completed!")
        print("=" * 60)
        print("\nIf motor didn't move, check:")
        print("  1. Wiring: PWM → GPIO18, DIR → GPIO23")
        print("  2. Motor driver power supply connected")
        print("  3. Motor driver enable pin (if present)")
        print("  4. Motor connections to driver output")
        print("  5. MD10S driver should have:")
        print("     - PWM input from GPIO18")
        print("     - DIR input from GPIO23")
        print("     - Ground connected to RPi ground")
        print("     - Separate power supply for motor")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    print("\nStarting in 3 seconds... Press Ctrl+C to cancel")
    try:
        time.sleep(3)
        test_gpio()
    except KeyboardInterrupt:
        print("\n\nTest cancelled by user")
        sys.exit(0)
