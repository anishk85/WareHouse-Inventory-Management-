#!/usr/bin/env python3
"""
Test ESP32 serial communication
"""
import serial
import time
import sys

def test_esp(port, baudrate=115200):
    print(f"\n=== Testing {port} ===")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  
        
    
        ser.reset_input_buffer()
        

        time.sleep(0.5)
        if ser.in_waiting:
            response = ser.readline().decode('utf-8').strip()
            print(f"ESP Response: {response}")
        

        print("Sending test command: V,100.0,100.0")
        ser.write(b"V,100.0,100.0\n")
        time.sleep(2)
        

        print("Stopping motors: V,0.0,0.0")
        ser.write(b"V,0.0,0.0\n")
        time.sleep(0.5)
        
        ser.close()
        print(f"✓ {port} test successful!")
        return True
        
    except Exception as e:
        print(f"✗ {port} test failed: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 test_esp_connection.py /dev/ttyUSB0 [/dev/ttyUSB1]")
        sys.exit(1)
    
    success = True
    for port in sys.argv[1:]:
        if not test_esp(port):
            success = False
    
    if success:
        print("\n✓ All ESP tests passed!")
    else:
        print("\n✗ Some ESP tests failed!")
        sys.exit(1)