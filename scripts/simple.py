#!/usr/bin/env python3
"""
Simplified ICM-20948 IMU Test Script
Tests using direct I2C register reads

Usage: python3 simple_test_icm20948.py
"""

import time
import sys

try:
    from smbus2 import SMBus
    print("✓ SMBus library loaded")
except ImportError:
    print("✗ Missing smbus2 library")
    print("Install: sudo pip3 install smbus2")
    sys.exit(1)

# ICM-20948 I2C addresses
ICM20948_ADDR = 0x69  # Default address (or 0x68)
I2C_BUS = 13  # Use bus 13 (change to 1, 13, or 14 as needed)

# ICM-20948 Registers
WHO_AM_I = 0x00
PWR_MGMT_1 = 0x06
ACCEL_XOUT_H = 0x2D
GYRO_XOUT_H = 0x33

def read_byte(bus, addr, reg):
    """Read a single byte from register"""
    return bus.read_byte_data(addr, reg)

def read_word(bus, addr, reg):
    """Read a 16-bit word (high and low bytes)"""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    # Convert to signed
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def test_connection():
    """Test I2C connection to ICM-20948"""
    print("\n" + "="*50)
    print("Testing ICM-20948 Connection")
    print("="*50)
    
    try:
        bus = SMBus(I2C_BUS)
        print(f"✓ I2C bus {I2C_BUS} opened")
        
        # Try to read WHO_AM_I register
        who_am_i = read_byte(bus, ICM20948_ADDR, WHO_AM_I)
        print(f"✓ WHO_AM_I register: 0x{who_am_i:02X}")
        
        if who_am_i == 0xEA:
            print("✓ ICM-20948 identified correctly!")
            return bus
        else:
            print(f"⚠ Unexpected WHO_AM_I value (expected 0xEA, got 0x{who_am_i:02X})")
            return bus
            
    except FileNotFoundError:
        print("✗ I2C device not found")
        print("  Enable I2C: sudo raspi-config → Interface Options → I2C")
        return None
    except OSError as e:
        print(f"✗ I2C communication error: {e}")
        print("  Check wiring and I2C address (try 0x68 if 0x69 fails)")
        return None

def initialize_sensor(bus):
    """Wake up and initialize the sensor"""
    print("\n" + "="*50)
    print("Initializing Sensor")
    print("="*50)
    
    try:
        # Wake up device (clear sleep bit)
        bus.write_byte_data(ICM20948_ADDR, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        print("✓ Sensor powered on")
        
        return True
        
    except Exception as e:
        print(f"✗ Initialization failed: {e}")
        return False

def read_sensors(bus, samples=5):
    """Read accelerometer and gyroscope data"""
    print("\n" + "="*50)
    print(f"Reading Sensor Data ({samples} samples)")
    print("="*50)
    
    try:
        for i in range(samples):
            # Read accelerometer (6 bytes)
            accel_x = read_word(bus, ICM20948_ADDR, ACCEL_XOUT_H)
            accel_y = read_word(bus, ICM20948_ADDR, ACCEL_XOUT_H + 2)
            accel_z = read_word(bus, ICM20948_ADDR, ACCEL_XOUT_H + 4)
            
            # Read gyroscope (6 bytes)
            gyro_x = read_word(bus, ICM20948_ADDR, GYRO_XOUT_H)
            gyro_y = read_word(bus, ICM20948_ADDR, GYRO_XOUT_H + 2)
            gyro_z = read_word(bus, ICM20948_ADDR, GYRO_XOUT_H + 4)
            
            # Convert to physical units
            # Accel: ±2g = 16384 LSB/g
            accel_scale = 16384.0
            ax = accel_x / accel_scale
            ay = accel_y / accel_scale
            az = accel_z / accel_scale
            
            # Gyro: ±250°/s = 131 LSB/(°/s)
            gyro_scale = 131.0
            gx = gyro_x / gyro_scale
            gy = gyro_y / gyro_scale
            gz = gyro_z / gyro_scale
            
            print(f"\nSample {i+1}:")
            print(f"  Accel: X={ax:7.3f}g  Y={ay:7.3f}g  Z={az:7.3f}g")
            print(f"  Gyro:  X={gx:7.1f}°/s Y={gy:7.1f}°/s Z={gz:7.1f}°/s")
            
            # Check if accelerometer detects gravity
            if i == samples - 1:
                total_g = (ax**2 + ay**2 + az**2)**0.5
                print(f"\n  Total acceleration: {total_g:.3f}g")
                if 0.9 < total_g < 1.1:
                    print("  ✓ Gravity detected correctly!")
                else:
                    print("  ⚠ Unexpected gravity reading")
            
            time.sleep(0.2)
        
        return True
        
    except Exception as e:
        print(f"✗ Read error: {e}")
        return False

def scan_i2c():
    """Scan for all I2C devices"""
    print("\n" + "="*50)
    print("Scanning I2C Bus")
    print("="*50)
    
    import os
    if os.geteuid() != 0:
        print("⚠ Not running as root (sudo)")
        print("  Some I2C operations may require sudo")
        print("  Try: sudo python3 simple_test_icm20948.py")
        return
    
    try:
        bus = SMBus(I2C_BUS)
        devices = []
        
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                devices.append(addr)
            except:
                pass
        
        if devices:
            print(f"✓ Found {len(devices)} device(s):")
            for addr in devices:
                print(f"  - 0x{addr:02X}")
        else:
            print("✗ No I2C devices found")
        
        bus.close()
        
    except Exception as e:
        print(f"✗ Scan failed: {e}")

def main():
    print("\n" + "="*50)
    print("ICM-20948 Simple Test")
    print("="*50)
    
    # First scan the bus
    scan_i2c()
    
    # Test connection
    bus = test_connection()
    if not bus:
        sys.exit(1)
    
    # Initialize
    if not initialize_sensor(bus):
        bus.close()
        sys.exit(1)
    
    # Read data
    if not read_sensors(bus):
        bus.close()
        sys.exit(1)
    
    bus.close()
    
    print("\n" + "="*50)
    print("✓ Test Complete!")
    print("="*50)
    print("\nYour ICM-20948 is working!\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
        sys.exit(0)
