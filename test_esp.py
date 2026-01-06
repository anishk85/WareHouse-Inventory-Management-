#!/usr/bin/env python3
"""
Manual Motor Control Interface
Interactive commands: forward, reverse, stop
"""

import serial
import time
import sys

class ESP32MotorController:
    def __init__(self, port='/dev/ttyUSB3', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connected = False
        
    def connect(self):
        """Establish serial connection to ESP32"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            time.sleep(2)  # Wait for ESP32 to reset
            self.connected = True
            print(f"✓ Connected to ESP32 on {self.port}")
            
            # Read any initial messages
            time.sleep(0.1)
            while self.ser.in_waiting > 0:
                msg = self.ser.readline().decode('utf-8').strip()
                if msg:
                    print(f"ESP32: {msg}")
            
            return True
            
        except serial.SerialException as e:
            print(f"✗ Error connecting to {self.port}: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.stop()
            time.sleep(0.1)
            self.ser.close()
            self.connected = False
            print("Disconnected from ESP32")
    
    def send_command(self, pwm, direction):
        """Send motor command to ESP32"""
        if not self.connected or not self.ser.is_open:
            print("Error: Not connected to ESP32")
            return False
        
        pwm = max(0, min(255, int(pwm)))
        direction = 1 if direction else 0
        
        cmd = f"{pwm},{direction}\n"
        
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            print(f"Sent: {cmd.strip()}")
            return True
            
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            return False
    
    def stop(self):
        """Stop the motor"""
        return self.send_command(0, 0)
    
    def forward(self, speed):
        """Drive motor forward"""
        return self.send_command(speed, 1)
    
    def reverse(self, speed):
        """Drive motor in reverse"""
        return self.send_command(speed, 0)


def print_menu():
    """Display command menu"""
    print("\n" + "="*50)
    print("MOTOR CONTROL MENU")
    print("="*50)
    print("Commands:")
    print("  f [speed]  - Forward (e.g., 'f 200' or 'f' for 150)")
    print("  r [speed]  - Reverse (e.g., 'r 150' or 'r' for 150)")
    print("  s          - Stop")
    print("  up         - Forward at 200")
    print("  down       - Reverse at 200")
    print("  q          - Quit")
    print("="*50)
    print("Speed range: 0-255 (default: 150)")
    print()


def main():
    print("Manual Motor Control Interface")
    print("-" * 50)
    
    # Connect to ESP32
    motor = ESP32MotorController(port='/dev/ttyUSB3')
    
    if not motor.connect():
        print("\nFailed to connect. Check:")
        print("  1. ESP32 is connected to /dev/ttyUSB3")
        print("  2. You have permissions: sudo usermod -a -G dialout $USER")
        print("  3. Port is correct: ls /dev/ttyUSB*")
        return
    
    print_menu()
    
    try:
        while True:
            # Get user input
            user_input = input("Enter command: ").strip().lower()
            
            if not user_input:
                continue
            
            parts = user_input.split()
            cmd = parts[0]
            
            # Parse speed if provided
            speed = 150  # Default speed
            if len(parts) > 1:
                try:
                    speed = int(parts[1])
                    speed = max(0, min(255, speed))
                except ValueError:
                    print(f"Invalid speed: {parts[1]}, using default {speed}")
            
            # Execute command
            if cmd == 'f' or cmd == 'forward':
                print(f"→ Moving FORWARD at speed {speed}")
                motor.forward(speed)
                
            elif cmd == 'r' or cmd == 'reverse':
                print(f"← Moving REVERSE at speed {speed}")
                motor.reverse(speed)
                
            elif cmd == 's' or cmd == 'stop':
                print("■ STOPPING motor")
                motor.stop()
                
            elif cmd == 'up':
                print("↑ Moving UP/FORWARD at speed 200")
                motor.forward(200)
                
            elif cmd == 'down':
                print("↓ Moving DOWN/REVERSE at speed 200")
                motor.reverse(200)
                
            elif cmd == 'q' or cmd == 'quit' or cmd == 'exit':
                print("\nExiting...")
                break
                
            elif cmd == 'h' or cmd == 'help' or cmd == '?':
                print_menu()
                
            else:
                print(f"Unknown command: {cmd}")
                print("Type 'h' for help")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user (Ctrl+C)")
    
    finally:
        print("\nStopping motor and disconnecting...")
        motor.disconnect()
        print("Program ended")


if __name__ == "__main__":
    main()