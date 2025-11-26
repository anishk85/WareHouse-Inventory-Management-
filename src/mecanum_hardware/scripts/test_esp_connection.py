#!/usr/bin/env python3
"""
ESP32 Stepper Controller Test Script (Updated for current firmware)
Tests communication with ESP32 firmware for mecanum robot

Usage:
    python3 test_esp32.py /dev/ttyUSB0
    python3 test_esp32.py /dev/ttyUSB0 --test-movement
"""

import serial
import time
import sys
import argparse

class ESP32Tester:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.esp_id = None
        
    def connect(self):
        """Open serial connection to ESP32"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")
            
            # Clear any garbage in the buffer
            self.ser.reset_input_buffer()
            time.sleep(0.5)
            
            print("\n--- Reading startup messages (3 seconds) ---")
            # Read startup messages
            startup_lines = self.read_all_available(timeout=3.0)
            
            for line in startup_lines:
                print(f"  {line}")
                
                # Extract ESP_ID from startup messages
                # Format: "ESP1:READY" or "ESP2:READY"
                if ":READY" in line:
                    try:
                        parts = line.split(":")
                        if parts[0].startswith("ESP"):
                            self.esp_id = int(parts[0].replace("ESP", ""))
                            print(f"  ✓ Detected ESP_ID: {self.esp_id}")
                    except:
                        pass
                elif "MAX_SPEED" in line:
                    print("  ✓ Configuration received")
            
            print("--- End of startup messages ---\n")
            
            if not self.esp_id:
                print("⚠ Could not detect ESP_ID from startup messages")
                print("  Trying STATUS command to verify connection...")
                
            return True
            
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"✓ Disconnected from {self.port}")
    
    def send_command(self, cmd):
        """Send command to ESP32"""
        if not self.ser or not self.ser.is_open:
            print("✗ Serial port not open")
            return False
        
        try:
            self.ser.write(f"{cmd}\n".encode())
            return True
        except Exception as e:
            print(f"✗ Failed to send command: {e}")
            return False
    
    def read_response(self, timeout=0.5):
        """Read response from ESP32"""
        if not self.ser or not self.ser.is_open:
            return None
        
        self.ser.timeout = timeout
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            return line if line else None
        except Exception as e:
            print(f"✗ Failed to read response: {e}")
            return None
    
    def read_all_available(self, timeout=0.5):
        """Read all available lines from ESP32"""
        lines = []
        self.ser.timeout = timeout
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"✗ Error reading: {e}")
                break
        
        return lines
    
    def test_status(self):
        """Test STATUS command"""
        print("\n=== Testing STATUS Command ===")
        self.send_command("STATUS")
        time.sleep(0.2)
        
        lines = self.read_all_available(timeout=0.5)
        
        if lines:
            print("✓ Response received:")
            for line in lines:
                print(f"  {line}")
            
            # Check for new firmware format: "ESP1:STATUS M1_current=..."
            status_found = any(":STATUS" in line and "M1_current=" in line for line in lines)
            
            if status_found:
                print("  ✓ Status format valid")
                return True
            else:
                print("  ⚠ Unexpected status format")
                return False
        else:
            print("✗ No response to STATUS command")
        return False
    
    def test_velocity(self, speed1=100.0, speed2=100.0):
        """Test velocity command"""
        print(f"\n=== Testing Velocity Command: V,{speed1},{speed2} ===")
        
        self.send_command(f"V,{speed1},{speed2}")
        time.sleep(0.2)
        
        # New firmware sends acknowledgment: "ESP1:M1=100.0,M2=100.0"
        lines = self.read_all_available(timeout=0.3)
        if lines:
            print("  Response:")
            for line in lines:
                print(f"    {line}")
                # Check for acknowledgment format
                if f":M1={speed1}" in line or f"M1={int(speed1)}" in line:
                    print("  ✓ Velocity command acknowledged")
                    return True
        
        # Verify by sending STATUS command
        print("  Verifying with STATUS...")
        self.send_command("STATUS")
        time.sleep(0.2)
        
        status_lines = self.read_all_available(timeout=0.5)
        if status_lines:
            for line in status_lines:
                print(f"    {line}")
                if "M1_target=" in line:
                    print("  ✓ Velocity command accepted (verified via STATUS)")
                    return True
        
        print("  ⚠ Could not verify velocity command")
        return False
    
    def test_stop(self):
        """Test STOP command"""
        print("\n=== Testing STOP Command ===")
        
        self.send_command("STOP")
        time.sleep(0.2)
        
        lines = self.read_all_available(timeout=0.5)
        if lines:
            print("✓ Response:")
            for line in lines:
                print(f"  {line}")
            
            # New firmware responds with "ESP1:STOP_ACTIVATED"
            if any("STOP_ACTIVATED" in line for line in lines):
                print("  ✓ Emergency stop confirmed")
                return True
        else:
            print("✗ No response to STOP command")
        return False
    
    def test_reset(self):
        """Test RESET command"""
        print("\n=== Testing RESET Command ===")
        
        self.send_command("RESET")
        time.sleep(0.2)
        
        lines = self.read_all_available(timeout=0.5)
        if lines:
            print("✓ Response:")
            for line in lines:
                print(f"  {line}")
            
            # Check for "ESP1:RESET_OK"
            if any("RESET_OK" in line for line in lines):
                print("  ✓ Reset confirmed")
                return True
        else:
            print("✗ No response to RESET command")
        return False
    
    def test_enable_disable(self):
        """Test ENABLE/DISABLE commands"""
        print("\n=== Testing ENABLE/DISABLE Commands ===")
        
        # Test ENABLE
        print("Testing ENABLE...")
        self.send_command("ENABLE")
        time.sleep(0.2)
        
        lines = self.read_all_available(timeout=0.3)
        enable_ok = any("ENABLED" in line for line in lines)
        if enable_ok:
            print("  ✓ ENABLE acknowledged")
        
        # Test DISABLE
        print("Testing DISABLE...")
        self.send_command("DISABLE")
        time.sleep(0.2)
        
        lines = self.read_all_available(timeout=0.3)
        disable_ok = any("DISABLED" in line for line in lines)
        if disable_ok:
            print("  ✓ DISABLE acknowledged")
        
        return enable_ok and disable_ok
    
    def test_movement_sequence(self):
        """Test complete movement sequence (MOTORS SHOULD BE OFF GROUND!)"""
        print("\n" + "="*60)
        print("=== MOVEMENT TEST SEQUENCE ===")
        print("="*60)
        print("\n⚠️  WARNING: MOTORS WILL SPIN!")
        print("    Make sure wheels are OFF THE GROUND!")
        input("\nPress ENTER to continue or Ctrl+C to abort...")
        
        sequences = [
            ("Forward slow", 200.0, 200.0, 2.0),
            ("Stop", 0.0, 0.0, 1.0),
            ("Backward slow", -200.0, -200.0, 2.0),
            ("Stop", 0.0, 0.0, 1.0),
            ("Spin right", 200.0, -200.0, 2.0),
            ("Stop", 0.0, 0.0, 1.0),
            ("Spin left", -200.0, 200.0, 2.0),
            ("Final stop", 0.0, 0.0, 1.0),
        ]
        
        for name, speed1, speed2, duration in sequences:
            print(f"\n→ {name}: M1={speed1}, M2={speed2} for {duration}s")
            
            # Send velocity
            self.send_command(f"V,{speed1},{speed2}")
            time.sleep(0.1)
            
            # Read acknowledgment
            lines = self.read_all_available(timeout=0.2)
            if lines:
                for line in lines:
                    print(f"  {line}")
            
            # Wait for duration
            time.sleep(duration)
            
            # Check status
            print("  Checking status...")
            self.send_command("STATUS")
            time.sleep(0.2)
            status_lines = self.read_all_available(timeout=0.5)
            for line in status_lines:
                print(f"    {line}")
        
        # Final emergency stop
        print("\n→ Emergency STOP")
        self.send_command("STOP")
        time.sleep(0.2)
        stop_lines = self.read_all_available(timeout=0.5)
        for line in stop_lines:
            print(f"  {line}")
        
        print("\n✓ Movement test sequence complete")
    
    def test_raw_communication(self):
        """Test raw serial communication - useful for debugging"""
        print("\n=== Testing Raw Communication ===")
        print("Checking if ESP32 is sending any data...")
        
        # Clear buffer
        self.ser.reset_input_buffer()
        
        # Wait and read any spontaneous messages (continuous STATUS reports)
        print("Listening for 2 seconds...")
        lines = self.read_all_available(timeout=2.0)
        
        if lines:
            print(f"✓ Received {len(lines)} line(s):")
            for line in lines[:5]:  # Show first 5 lines
                print(f"  {line}")
            if len(lines) > 5:
                print(f"  ... and {len(lines)-5} more lines")
            return True
        else:
            print("✗ No data received")
            print("  Troubleshooting tips:")
            print("  - Check if ESP32 is powered on")
            print("  - Verify correct serial port (try: ls -l /dev/ttyUSB*)")
            print("  - Check USB cable connection")
            print("  - Verify baud rate matches firmware (115200)")
            print("  - Try pressing reset button on ESP32")
            return False
    
    def run_basic_tests(self):
        """Run all basic communication tests"""
        print("\n" + "="*60)
        print("=== ESP32 BASIC COMMUNICATION TESTS ===")
        print("="*60)
        
        # First check raw communication
        raw_ok = self.test_raw_communication()
        
        if not raw_ok:
            print("\n⚠️ Raw communication test failed!")
            print("   Skipping other tests until basic communication works.")
            return False
        
        results = {
            "RAW_COMM": raw_ok,
            "STATUS": self.test_status(),
            "VELOCITY": self.test_velocity(100.0, -100.0),
            "STOP": self.test_stop(),
            "RESET": self.test_reset(),
            "ENABLE/DISABLE": self.test_enable_disable(),
        }
        
        print("\n" + "="*60)
        print("=== TEST RESULTS ===")
        print("="*60)
        for test, passed in results.items():
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"{test:20s} {status}")
        
        total = len(results)
        passed = sum(results.values())
        print(f"\nPassed: {passed}/{total}")
        
        return all(results.values())

def print_troubleshooting_guide():
    """Print comprehensive troubleshooting guide"""
    print("\n" + "="*60)
    print("=== TROUBLESHOOTING GUIDE ===")
    print("="*60)
    print("""
If you're getting no response from ESP32:

1. **Check Hardware Connections:**
   - Is ESP32 powered on? (LED should be lit)
   - Is USB cable properly connected?
   - Try a different USB cable (some cables are power-only)
   - Check: ls -l /dev/ttyUSB*  (does your port exist?)
   - Check: dmesg | tail  (any USB connection messages?)

2. **Check Serial Port:**
   - Wrong port? Try: ls /dev/ttyUSB* or ls /dev/ttyACM*
   - Permissions? Run: sudo chmod 666 /dev/ttyUSB0
   - Or add user to dialout group: sudo usermod -a -G dialout $USER
   - Port in use? Check: lsof /dev/ttyUSB0

3. **Check ESP32 Firmware:**
   - Is firmware uploaded? Try pressing reset button
   - Correct baud rate? Firmware uses 115200
   - Open Arduino Serial Monitor to verify ESP32 output
   - Look for startup message: "ESP1:READY" or "ESP2:READY"

4. **Check for Binary/Garbage Data:**
   - ESP32 might be in bootloader mode
   - Press reset button on ESP32
   - Re-upload firmware if needed

5. **Test with Arduino Serial Monitor:**
   - Open Tools -> Serial Monitor in Arduino IDE
   - Set baud to 115200
   - You should see "ESP1:READY" or "ESP2:READY" on startup
   - Try typing: STATUS
   - Should get: "ESP1:STATUS M1_current=0.0 M1_target=0.0 ..."

6. **Common Issues:**
   - UTF-8 decode errors = ESP32 sending binary data (press reset)
   - No response = Wrong port, wrong baud, or no firmware
   - Timeout = Communication working but ESP32 not responding
   - Python serial module issues: pip3 install pyserial

7. **Expected Firmware Responses:**
   - Startup: "ESP1:READY", "ESP1:MAX_SPEED=30000 steps/sec", etc.
   - STATUS command: "ESP1:STATUS M1_current=... M1_target=... M1_steps=..."
   - Velocity command: "ESP1:M1=100.0,M2=-100.0"
   - STOP command: "ESP1:STOP_ACTIVATED"
   - Continuous status reports at 50Hz (one line every 20ms)
""")

def main():
    parser = argparse.ArgumentParser(
        description='Test ESP32 stepper controller firmware',
        epilog='Example: python3 test_esp32.py /dev/ttyUSB0'
    )
    parser.add_argument('port', nargs='?', help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Serial baudrate (default: 115200)')
    parser.add_argument('--test-movement', action='store_true', help='Run movement test sequence (MOTORS SHOULD BE OFF GROUND!)')
    parser.add_argument('--troubleshoot', action='store_true', help='Show troubleshooting guide')
    
    args = parser.parse_args()
    
    # Show troubleshooting guide if requested
    if args.troubleshoot:
        print_troubleshooting_guide()
        return
    
    # Port is required if not showing troubleshooting
    if not args.port:
        parser.print_help()
        print("\n✗ Error: Serial port is required")
        print("   Example: python3 test_esp32.py /dev/ttyUSB0")
        print("   Or use --troubleshoot for help")
        sys.exit(1)
    
    # Create tester
    tester = ESP32Tester(args.port, args.baudrate)
    
    try:
        # Connect
        if not tester.connect():
            sys.exit(1)
        
        # Run basic tests
        all_passed = tester.run_basic_tests()
        
        # Run movement test if requested
        if args.test_movement:
            tester.test_movement_sequence()
        
        # Disconnect
        tester.disconnect()
        
        # Exit with appropriate code
        sys.exit(0 if all_passed else 1)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        tester.disconnect()
        sys.exit(130)
    except Exception as e:
        print(f"\n✗ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        tester.disconnect()
        sys.exit(1)

if __name__ == "__main__":
    main()