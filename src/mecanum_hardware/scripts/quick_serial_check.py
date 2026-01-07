#!/usr/bin/env python3
"""
Quick Serial Port Checker
Quickly diagnose ESP32 serial communication issues
"""

import serial
import time
import sys
import os

def check_port_exists(port):
    """Check if serial port exists"""
    if os.path.exists(port):
        print(f"✓ Port {port} exists")
        # Check permissions
        if os.access(port, os.R_OK | os.W_OK):
            print(f"✓ Port {port} has read/write permissions")
            return True
        else:
            print(f"✗ Port {port} exists but no read/write permissions")
            print(f"  Fix: sudo chmod 666 {port}")
            print(f"  Or: sudo usermod -a -G dialout $USER (then logout/login)")
            return False
    else:
        print(f"✗ Port {port} does not exist")
        print("  Available ports:")
        os.system("ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo '  No USB serial ports found'")
        return False

def listen_to_port(port, duration=5):
    """Listen to port for any data"""
    print(f"\nListening to {port} for {duration} seconds...")
    print("(ESP32 should send STATS messages every 5 seconds)")
    print("-" * 60)
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        print(f"✓ Successfully opened {port}")
        
        # Clear buffer
        ser.reset_input_buffer()
        
        start_time = time.time()
        line_count = 0
        
        while (time.time() - start_time) < duration:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        line_count += 1
                        print(f"[{time.time()-start_time:.1f}s] {line}")
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"Error reading: {e}")
        
        print("-" * 60)
        if line_count > 0:
            print(f"✓ Received {line_count} line(s) - ESP32 is communicating!")
        else:
            print("✗ No data received")
            print("\nTroubleshooting:")
            print("  1. Press the RESET button on ESP32")
            print("  2. Check if ESP32 power LED is on")
            print("  3. Try a different USB cable")
            print("  4. Check firmware is uploaded (use Arduino Serial Monitor)")
            print("  5. Verify baud rate is 115200 in firmware")
        
        ser.close()
        return line_count > 0
        
    except serial.SerialException as e:
        print(f"✗ Failed to open port: {e}")
        return False

def test_command(port, command):
    """Send a command and wait for response"""
    print(f"\nTesting command: {command}")
    print("-" * 60)
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        
        # Clear buffer
        ser.reset_input_buffer()
        time.sleep(0.1)
        
        # Send command
        ser.write(f"{command}\n".encode())
        print(f"Sent: {command}")
        
        # Read response
        time.sleep(0.3)
        lines = []
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                lines.append(line)
        
        if lines:
            print("Response:")
            for line in lines:
                print(f"  {line}")
            print("✓ Command successful")
        else:
            print("✗ No response")
        
        print("-" * 60)
        ser.close()
        return len(lines) > 0
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def main():
    print("="*60)
    print("ESP32 Quick Serial Port Checker")
    print("="*60)
    
    if len(sys.argv) < 2:
        print("\nUsage: python3 quick_serial_check.py /dev/ttyUSB0")
        print("\nAvailable ports:")
        os.system("ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo 'No USB serial ports found'")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print(f"\nChecking: {port}")
    print("="*60)
    
    # Step 1: Check if port exists and has permissions
    if not check_port_exists(port):
        sys.exit(1)
    
    # Step 2: Listen for any data
    if not listen_to_port(port, duration=6):
        print("\n⚠️  No data received. ESP32 might not be running firmware.")
        print("   Try pressing the RESET button on ESP32 and run this again.")
        sys.exit(1)
    
    # Step 3: Test STATUS command
    if test_command(port, "STATUS"):
        print("\n✓✓✓ ESP32 is working correctly! ✓✓✓")
    else:
        print("\n⚠️  ESP32 sends data but doesn't respond to STATUS")
        print("   This might indicate wrong firmware or firmware issue")
    
    # Step 4: Test STOP command
    test_command(port, "STOP")
    
    print("\n" + "="*60)
    print("Check complete!")
    print("="*60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(130)
