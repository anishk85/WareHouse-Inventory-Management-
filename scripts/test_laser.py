#!/usr/bin/env python3
# filepath: /root/kl200_laser_sensor.py

"""
Correct implementation for PRO RANGE-KL200-NPN Laser Distance Sensor
Uses UART serial communication instead of pulse timing
"""

import serial
import time
import struct

# Wiring for UART mode:
# Red (VCC)    -> 5V
# Blue (GND)   -> GND  
# Yellow (TX)  -> GPIO 15 (RXD - Pin 10) - Sensor transmits TO Pi
# Black (RX)   -> GPIO 14 (TXD - Pin 8)  - Sensor receives FROM Pi

class KL200Sensor:
    def __init__(self, port='/dev/serial0', baudrate=9600):
        """
        Initialize KL200 sensor
        Default baud rate is 9600 (can be 2400-128000)
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        time.sleep(0.1)  # Let serial stabilize
        
    def read_distance(self):
        """
        Read distance from sensor in auto-upload mode
        Returns distance in mm, or None if error
        """
        if self.ser.in_waiting >= 9:
            data = self.ser.read(9)
            
            # Verify checksum
            if len(data) == 9:
                xor_check = data[0]
                for i in range(1, 8):
                    xor_check ^= data[i]
                
                if xor_check == data[8]:
                    # Extract distance (bytes 5-6, big endian)
                    distance_mm = struct.unpack('>H', data[5:7])[0]
                    return distance_mm
        return None
    
    def manual_query(self, address=0xFFFF):
        """
        Manually request distance reading
        Send command: 62 33 09 FF FF 00 00 00 XOR
        """
        cmd = bytearray([0x62, 0x33, 0x09, 
                        (address >> 8) & 0xFF, address & 0xFF,
                        0x00, 0x00, 0x00])
        
        # Calculate XOR checksum
        xor = 0
        for b in cmd:
            xor ^= b
        cmd.append(xor)
        
        self.ser.write(cmd)
        time.sleep(0.05)
        
        return self.read_distance()
    
    def configure_auto_upload(self, interval_100ms=10):
        """
        Configure sensor to auto-upload distance data
        interval_100ms: 1-100 (100ms to 10s intervals)
        """
        # Command: 62 35 09 FF FF 00 [interval] 00 XOR
        cmd = bytearray([0x62, 0x35, 0x09, 0xFF, 0xFF, 
                        0x00, interval_100ms & 0xFF, 0x00])
        xor = 0
        for b in cmd:
            xor ^= b
        cmd.append(xor)
        
        self.ser.write(cmd)
        time.sleep(0.1)
        
        # Read ACK
        if self.ser.in_waiting >= 9:
            ack = self.ser.read(9)
            return ack[7] == 0x66  # Success byte
        return False
    
    def close(self):
        self.ser.close()


def main():
    print("=" * 60)
    print("  PRO RANGE-KL200-NPN Laser Distance Sensor")
    print("  UART Serial Communication Mode")
    print("=" * 60)
    print()
    
    # Enable UART on RPi5
    print("Make sure you've enabled UART:")
    print("  sudo raspi-config")
    print("  Interface Options -> Serial Port")
    print("  Login shell: NO, Hardware: YES")
    print()
    
    try:
        sensor = KL200Sensor()
        print("✓ Serial port opened")
        
        # Configure for auto-upload every 500ms
        if sensor.configure_auto_upload(interval_100ms=5):
            print("✓ Configured for auto-upload mode (500ms interval)")
        
        print("\nReading distances (Ctrl+C to stop):")
        print("-" * 60)
        
        count = 0
        while True:
            distance_mm = sensor.read_distance()
            
            if distance_mm is not None:
                count += 1
                distance_cm = distance_mm / 10.0
                distance_m = distance_mm / 1000.0
                
                # Check if in valid range (10mm - 4000mm)
                if 10 <= distance_mm <= 4000:
                    print(f"[{count:4d}] ✓ {distance_mm:4d} mm | "
                          f"{distance_cm:6.1f} cm | {distance_m:5.3f} m")
                else:
                    print(f"[{count:4d}] ⚠ Out of range: {distance_mm} mm")
            
            time.sleep(0.05)  # Small delay between reads
            
    except serial.SerialException as e:
        print(f"\n❌ Serial Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check wiring (Yellow->Pin10, Black->Pin8)")
        print("2. Enable UART: sudo raspi-config")
        print("3. Disable serial console: sudo systemctl disable serial-getty@serial0")
        print("4. Add to /boot/firmware/config.txt: enable_uart=1")
        
    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
        
    finally:
        sensor.close()
        print("✓ Serial port closed")

if __name__ == '__main__':
    main()
