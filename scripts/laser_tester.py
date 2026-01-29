import serial
import time
import struct
import sys
from typing import Optional

# --- Configuration ---
# IMPORTANT: Use the /dev/serial0 symbolic link for maximum compatibility
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 9600
PACKET_SIZE = 9

# Distance Sensor Commands
# Command structure: [Cmd1][Cmd2][Len][AddrH][AddrL][DataH][DataL][Reserved][Check]
SET_UART_MODE = b'\x61\x30\x09\xFF\xFF\x00\x01\x00\x59' # Checksum: 59 (Switches to UART)
SET_AUTO_UPLOAD = b'\x62\x34\x09\xFF\xFF\x00\x01\x00\x5E' # Checksum: 5E (Sets continuous output)

# Global buffer to hold incoming serial data and handle misalignment
# Using bytearray allows efficient appending and slicing
data_buffer = bytearray()

# --- Checksum and Parsing Functions ---

def calculate_checksum(data: bytes) -> int:
    """Calculates the XOR checksum of a byte sequence."""
    checksum = 0
    # XOR all bytes except the last one (which is the expected checksum)
    for byte in data[:-1]:
        checksum ^= byte
    return checksum

def parse_distance_packet(packet: bytes) -> Optional[int]:
    """
    Validates the packet checksum and extracts the distance value.
    Returns distance in mm, or None if the packet is invalid.
    """
    if len(packet) != PACKET_SIZE:
        return None

    # Calculate and check checksum
    calc_checksum = calculate_checksum(packet)
    received_checksum = packet[-1]
    
    if calc_checksum == received_checksum:
        # Distance data is in bytes 5 and 6 (High Byte, Low Byte)
        distance_bytes = packet[5:7]
        # '>H' means Big-Endian (>) Unsigned Short (H)
        distance = struct.unpack('>H', distance_bytes)[0]
        return distance
    else:
        # Checksum failed, print debug info
        hex_packet = ' '.join(f'{b:02X}' for b in packet)
        print(f"Checksum Error: Packet={hex_packet} | Calc={calc_checksum:02X}, Recv={received_checksum:02X}")
        return None

# --- Main Logic ---

def setup_serial():
    """Initializes the serial port and configures the sensor."""
    print(f"Starting PRO RANGE KL200 Sensor on {SERIAL_PORT}...")
    
    try:
        # Initialize serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.flushInput()
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        print("ACTION: Check port name, wiring, and `raspi-config` settings.")
        sys.exit(1)

    # 1. Configure sensor to UART mode
    print("Configuring sensor to UART mode...")
    ser.write(SET_UART_MODE)
    time.sleep(2.0) # Increased delay for stability

    # 2. Enable Auto-Upload
    print("Enabling Auto-Upload (Continuous Data)...")
    ser.write(SET_AUTO_UPLOAD)
    time.sleep(2.0) # Increased delay for stability
    
    ser.flushInput()
    print("Setup Complete. Reading distance...")
    
    return ser

def main():
    global data_buffer
    ser = setup_serial()

    # --- Main Loop ---
    try:
        while True:
            # 1. Read all available data into the global buffer
            if ser.in_waiting > 0:
                raw_data = ser.read(ser.in_waiting)
                data_buffer.extend(raw_data)
                
            # 2. Check the buffer for a full packet
            if len(data_buffer) >= PACKET_SIZE:
                
                # Take the first PACKET_SIZE bytes
                current_packet = data_buffer[:PACKET_SIZE]
                
                # Attempt to parse and validate
                distance = parse_distance_packet(current_packet)
                
                if distance is not None:
                    # Packet is valid! Print distance and remove the packet from the buffer
                    print(f"Distance: {distance} mm")
                    del data_buffer[:PACKET_SIZE]
                    
                else:
                    # Packet is INVALID (Checksum Failure)
                    # This means we are misaligned. Discard the first byte (byte 0) 
                    # to try and re-align the stream on the next packet start.
                    print("ALIGNMENT: Checksum failure, discarding 1 byte for realignment.")
                    del data_buffer[0]
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting program.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
