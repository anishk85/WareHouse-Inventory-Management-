#!/usr/bin/env python3
"""
ESP32 Motor Controller Debug Tool
Tests connection and communication
"""

import serial
import time
import sys

def list_ports():
    """List all available serial ports"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        print("\n=== Available Serial Ports ===")
        if not ports:
            print("No ports found!")
        for port in ports:
            print(f"  {port.device}: {port.description}")
        print()
        return [p.device for p in ports]
    except ImportError:
        print("Install pyserial: pip3 install pyserial")
        return []

def test_connection(port, baudrate=115200):
    """Test basic connection to ESP32"""
    print(f"\n=== Testing Connection: {port} at {baudrate} baud ===")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"✓ Port opened successfully")
        
        # Wait for ESP32 reset
        print("Waiting for ESP32 to initialize (2 seconds)...")
        time.sleep(2)
        
        # Check for any startup messages
        print("\n--- Reading startup messages ---")
        for i in range(10):
            if ser.in_waiting > 0:
                msg = ser.readline().decode('utf-8', errors='ignore').strip()
                if msg:
                    print(f"ESP32: {msg}")
            time.sleep(0.1)
        
        # Test sending commands
        print("\n--- Testing Commands ---")
        
        test_commands = [
            ("0,0", "Stop (0,0)"),
            ("100,1", "Forward 100"),
            ("150,1", "Forward 150"),
            ("0,0", "Stop again"),
        ]
        
        for cmd, desc in test_commands:
            print(f"\nSending: '{cmd}' ({desc})")
            ser.write(f"{cmd}\n".encode('utf-8'))
            ser.flush()
            print(f"  Bytes written: {len(cmd) + 1}")
            
            # Wait and check for response
            time.sleep(0.3)
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"  ESP32 Response: {response}")
        
        # Final stop
        print("\n--- Sending final STOP ---")
        ser.write(b"0,0\n")
        ser.flush()
        time.sleep(0.2)
        
        ser.close()
        print("\n✓ Test completed successfully")
        return True
        
    except serial.SerialException as e:
        print(f"✗ Serial Error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected Error: {e}")
        return False

def check_permissions(port):
    """Check if user has permission to access port"""
    import os
    print(f"\n=== Checking Permissions for {port} ===")
    
    if not os.path.exists(port):
        print(f"✗ Port {port} does not exist!")
        return False
    
    if os.access(port, os.R_OK | os.W_OK):
        print(f"✓ You have read/write access to {port}")
        return True
    else:
        print(f"✗ No permission to access {port}")
        print(f"\nFix with: sudo usermod -a -G dialout $USER")
        print(f"Then logout and login again, or use: sudo chmod 666 {port}")
        return False

if __name__ == "__main__":
    print("=" * 50)
    print("ESP32 Motor Controller Diagnostic Tool")
    print("=" * 50)
    
    # Step 1: List all ports
    available_ports = list_ports()
    
    # Step 2: Check the target port
    target_port = "/dev/ttyUSB3"
    
    if len(sys.argv) > 1:
        target_port = sys.argv[1]
        print(f"Using port from command line: {target_port}")
    
    # Step 3: Check permissions
    check_permissions(target_port)
    
    # Step 4: Test connection
    print("\nPress Enter to test connection, or Ctrl+C to exit...")
    try:
        input()
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)
    
    test_connection(target_port)
    
    print("\n" + "=" * 50)
    print("Diagnostic complete!")
    print("=" * 50)