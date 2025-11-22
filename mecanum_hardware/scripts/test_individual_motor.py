#!/usr/bin/env python3
"""
Test individual motors on ESP32
"""
import serial
import time
import sys

def test_motor(port, motor_num, speed=500.0, duration=2.0):
    """
    Test individual motor
    motor_num: 1 or 2
    speed: steps/sec
    """
    print(f"\nTesting {port} - Motor {motor_num} at {speed} steps/sec")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        
        # Clear buffer
        ser.reset_input_buffer()
        
        # Send test command
        if motor_num == 1:
            cmd = f"V,{speed},0.0\n"
        else:
            cmd = f"V,0.0,{speed}\n"
        
        print(f"Sending: {cmd.strip()}")
        ser.write(cmd.encode())
        
        time.sleep(duration)
        
        # Stop
        ser.write(b"V,0.0,0.0\n")
        print("Motor stopped")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 test_individual_motor.py <port> <motor_num> [speed] [duration]")
        print("Example: python3 test_individual_motor.py /dev/ttyUSB0 1 500 2")
        sys.exit(1)
    
    port = sys.argv[1]
    motor_num = int(sys.argv[2])
    speed = float(sys.argv[3]) if len(sys.argv) > 3 else 500.0
    duration = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    
    test_motor(port, motor_num, speed, duration)