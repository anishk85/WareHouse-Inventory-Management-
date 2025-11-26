# ESP32 Communication Troubleshooting Guide

## Issue Summary

You were experiencing **no response** from the ESP32 after initially seeing UTF-8 decode errors. This is a common issue with several possible causes.

## Root Causes Identified

### 1. **Test Script Mismatch with Firmware**
The original test script expected responses that the ESP32 firmware doesn't provide:
- **Velocity commands** (`V,speed1,speed2`) - Firmware accepts these **silently** (no acknowledgment)
- **ENABLE/DISABLE commands** - Not implemented in firmware
- **RESET command** - Not implemented in firmware
- **Response format** - Test expected different format than firmware sends

### 2. **UTF-8 Decode Errors**
When you saw errors like:
```
✗ Failed to read response: 'utf-8' codec can't decode byte 0xe0 in position 1: invalid continuation byte
```

**Cause:** ESP32 was sending binary data or was in bootloader mode during startup.

**Fix:** 
- Added `errors='ignore'` to decode() calls
- Clear input buffer before reading
- Wait for ESP32 to fully boot

### 3. **No Response After First Run**
After pyserial was installed, you got no responses.

**Possible causes:**
1. ESP32 is not running the firmware (stuck in bootloader)
2. Wrong serial port
3. Serial port permission issues
4. ESP32 needs reset
5. Port is held open by another process

## ESP32 Firmware Protocol

The firmware (`esp32_stepper_controller.ino`) implements these commands:

### Commands That GET Responses:

1. **STATUS** - Prints multi-line status:
   ```
   --- STATUS ---
   ESP_ID: 1
   FL: target=0.0 current=0.0 dir=FWD
   FR: target=0.0 current=0.0 dir=FWD
   -------------
   ```

2. **STOP** - Prints:
   ```
   EMERGENCY STOP
   ```

3. **Startup Messages** - On boot/reset:
   ```
   ========================================
   ESP32 Mecanum Stepper Controller
   ESP_ID: 1
   Position: FRONT (FL + FR)
   ...
   Ready!
   ========================================
   ```

4. **Stats Messages** - Every 5 seconds automatically:
   ```
   STATS: ESP1 | FL=0.0 FR=0.0 | Loops/sec=12345
   ```

### Commands That DON'T Get Responses:

1. **V,speed1,speed2** - Velocity command (accepted silently)
   - Note: There's commented code in firmware that could echo this
   - Motors will start moving based on these speeds

## Diagnostic Steps

### Step 1: Check Hardware
```bash
# List USB serial devices
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*

# Check USB connection messages
dmesg | tail

# Check if port is in use
lsof /dev/ttyUSB0
```

### Step 2: Check Permissions
```bash
# Quick fix (temporary)
sudo chmod 666 /dev/ttyUSB0

# Permanent fix
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Step 3: Test with Updated Script
```bash
# Basic test (now matches firmware protocol)
python3 test_esp_connection.py /dev/ttyUSB0

# Show troubleshooting guide
python3 test_esp_connection.py --troubleshoot

# Test with movement (wheels off ground!)
python3 test_esp_connection.py /dev/ttyUSB0 --test-movement
```

### Step 4: Manual Serial Test
```bash
# Install screen if not available
sudo apt-get install screen

# Connect to ESP32
screen /dev/ttyUSB0 115200

# You should see STATS messages every 5 seconds
# Type: STATUS (then Enter)
# Type: STOP (then Enter)
# Type: V,100,100 (then Enter) - motors will spin if connected!

# Exit screen: Ctrl+A then K then Y
```

### Step 5: Arduino Serial Monitor
1. Open Arduino IDE
2. Tools -> Serial Monitor
3. Set baud to 115200
4. Press reset button on ESP32
5. Should see startup messages
6. Type commands (STATUS, STOP, etc.)

## Common Issues & Solutions

### Issue: "No module named 'serial'"
```bash
pip3 install pyserial
# NOT: pip3 install serial (wrong package!)
```

### Issue: "Permission denied" on /dev/ttyUSB0
```bash
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group (see above)
```

### Issue: UTF-8 decode errors
**Cause:** ESP32 sending binary data (bootloader mode)

**Solution:**
1. Press reset button on ESP32
2. Or re-upload firmware
3. Script now handles this gracefully with `errors='ignore'`

### Issue: No response at all
**Possible causes:**
1. **Wrong port** - Check `ls /dev/ttyUSB*`
2. **ESP32 not powered** - Check power LED
3. **No firmware** - Upload firmware via Arduino IDE
4. **Wrong baud rate** - Should be 115200
5. **Port held by another process** - Check `lsof /dev/ttyUSB0`

**Solution:**
1. Try pressing reset button on ESP32
2. Disconnect and reconnect USB cable
3. Check if Arduino Serial Monitor can connect
4. Re-upload firmware if needed

### Issue: Port disappears after connecting
**Cause:** ESP32 resetting when serial port opens (normal behavior)

**Solution:** Script now waits 2 seconds for ESP32 to boot

## Testing Strategy

### 1. Basic Connectivity Test
```bash
python3 test_esp_connection.py /dev/ttyUSB0
```
This will:
- Connect and read startup messages
- Test raw communication (wait for STATS)
- Test STATUS command
- Test VELOCITY command (verify via STATUS)
- Test STOP command

### 2. Manual Command Test
Use Arduino Serial Monitor or `screen` to manually verify ESP32 is responding.

### 3. Movement Test (Wheels OFF Ground!)
```bash
python3 test_esp_connection.py /dev/ttyUSB0 --test-movement
```

## Next Steps

If still not working:

1. **Verify firmware is uploaded:**
   - Open Arduino IDE
   - Open firmware: `firmware/esp32_stepper_controller/esp32_stepper_controller.ino`
   - Verify board selection: ESP32 Dev Module
   - Upload firmware
   - Check serial monitor for startup messages

2. **Check ESP_ID configuration:**
   - In firmware, line ~15: `const int ESP_ID = 1;`
   - Should be 1 for front ESP, 2 for back ESP
   - Re-upload if changed

3. **Enable debug output in firmware:**
   - Uncomment the CMD echo lines around line 228:
   ```cpp
   // Optional: Echo command for debugging
   Serial.print("CMD: ");
   Serial.print(motor1.name);
   Serial.print("=");
   Serial.print(motor1.target_steps_per_sec);
   Serial.print(" ");
   Serial.print(motor2.name);
   Serial.print("=");
   Serial.println(motor2.target_steps_per_sec);
   ```

4. **Contact for help with:**
   - Complete error output
   - Result of `dmesg | tail`
   - Result of Arduino Serial Monitor test
   - Firmware version/modifications

## Summary of Script Improvements

The updated `test_esp_connection.py` now:
1. ✅ Handles UTF-8 decode errors gracefully
2. ✅ Waits for ESP32 boot properly
3. ✅ Reads multi-line responses correctly
4. ✅ Matches actual firmware protocol
5. ✅ Tests raw communication first
6. ✅ Provides detailed troubleshooting tips
7. ✅ Includes `--troubleshoot` flag
8. ✅ Removes tests for unimplemented commands
9. ✅ Verifies velocity commands via STATUS
10. ✅ Reads automatic STATS messages

Run the updated script and it should work much better!
