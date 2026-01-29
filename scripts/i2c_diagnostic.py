#!/usr/bin/env python3
"""
I2C Bus Diagnostic Tool
Helps identify which I2C bus to use
"""

import os
import sys

try:
    from smbus2 import SMBus
except ImportError:
    print("Install: sudo pip3 install smbus2")
    sys.exit(1)

print("="*60)
print("I2C Bus Diagnostic Tool")
print("="*60)

# Check Pi model
try:
    with open('/proc/device-tree/model', 'r') as f:
        model = f.read().strip('\x00')
        print(f"\nRaspberry Pi Model: {model}")
except:
    print("\nCannot determine Pi model")

# List available I2C devices
print("\nAvailable I2C devices:")
i2c_devices = [f for f in os.listdir('/dev') if f.startswith('i2c-')]
for dev in sorted(i2c_devices):
    print(f"  - /dev/{dev}")

# Test each bus
print("\n" + "="*60)
print("Testing Each I2C Bus")
print("="*60)

for dev in sorted(i2c_devices):
    bus_num = int(dev.split('-')[1])
    print(f"\n--- Bus {bus_num} ---")
    
    try:
        bus = SMBus(bus_num)
        
        # Try to scan for devices
        devices = []
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                devices.append(addr)
            except:
                pass
        
        bus.close()
        
        if len(devices) == 0:
            print(f"  Status: Empty (no devices)")
        elif len(devices) > 100:
            print(f"  Status: Invalid (shows {len(devices)} devices - likely not a real bus)")
        else:
            print(f"  Status: Active ({len(devices)} device(s) found)")
            print(f"  Addresses: {', '.join([f'0x{a:02X}' for a in devices])}")
            
            # Check for ICM-20948
            if 0x68 in devices or 0x69 in devices:
                print(f"  *** ICM-20948 DETECTED on bus {bus_num} ***")
    
    except PermissionError:
        print(f"  Status: Permission denied (try with sudo)")
    except Exception as e:
        print(f"  Status: Error - {e}")

print("\n" + "="*60)
print("Recommendations")
print("="*60)

print("""
For ICM-20948 on Raspberry Pi:

Standard I2C pins (Bus 1):
  - Pin 1  (3.3V)    → VCC
  - Pin 3  (GPIO 2)  → SDA  
  - Pin 5  (GPIO 3)  → SCL
  - Pin 6  (GND)     → GND

If bus 1 is empty, your wiring may be incorrect.

Troubleshooting:
1. Verify physical connections
2. Check if sensor LED is lit (if it has one)
3. Measure voltage at VCC pin (should be 3.3V)
4. Try different jumper wires
5. Ensure sensor is not damaged
""")

print("\nTo test with a specific bus, run:")
print("  sudo i2cdetect -y <bus_number>")
print()
