#!/usr/bin/env python3
"""
ROS2 Node for PRO RANGE-KL200-NPN Laser Distance Sensor
Publishes distance measurements as sensor_msgs/Range

Topic: /laser_distance (sensor_msgs/Range)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import struct
import time


class KL200LaserNode(Node):
    def __init__(self):
        super().__init__('kl200_laser_sensor')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'laser_sensor')
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('auto_upload_interval', 5)  # 500ms (5 x 100ms)
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        upload_interval = self.get_parameter('auto_upload_interval').value
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            time.sleep(0.1)
            self.get_logger().info(f'Serial port {port} opened at {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Configure sensor for auto-upload mode
        if self.configure_auto_upload(upload_interval):
            self.get_logger().info(f'Configured auto-upload mode ({upload_interval*100}ms interval)')
        else:
            self.get_logger().warn('Failed to configure auto-upload, using manual query mode')
            self.manual_mode = True
        
        # Create publisher
        self.publisher = self.create_publisher(Range, 'laser_distance', 10)
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Statistics
        self.measurement_count = 0
        self.error_count = 0
        
        self.get_logger().info('KL200 Laser Sensor Node initialized')
        self.get_logger().info(f'Publishing on topic: laser_distance at {publish_rate} Hz')
    
    def configure_auto_upload(self, interval_100ms):
        """
        Configure sensor to auto-upload distance data
        interval_100ms: 1-100 (100ms to 10s intervals)
        Command: 62 35 09 FF FF 00 [interval] 00 XOR
        """
        cmd = bytearray([0x62, 0x35, 0x09, 0xFF, 0xFF, 
                        0x00, interval_100ms & 0xFF, 0x00])
        
        # Calculate XOR checksum
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
    
    def read_distance(self):
        """
        Read distance from sensor in auto-upload mode
        Returns distance in meters, or None if error
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
                    # Convert mm to meters
                    return distance_mm / 1000.0
                else:
                    self.error_count += 1
                    self.get_logger().debug('Checksum error')
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
    
    def timer_callback(self):
        """Timer callback to read sensor and publish Range message"""
        distance = self.read_distance()
        
        if distance is not None:
            # Check if in valid range (0.01m - 4.0m)
            if 0.01 <= distance <= 4.0:
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                
                # Range message fields
                msg.radiation_type = Range.INFRARED  # Laser is typically infrared
                msg.field_of_view = 0.436  # ~25 degrees receive angle in radians
                msg.min_range = 0.01  # 10mm in meters
                msg.max_range = 4.0   # 4000mm in meters
                msg.range = distance
                
                self.publisher.publish(msg)
                self.measurement_count += 1
                
                # Log every 100 measurements
                if self.measurement_count % 100 == 0:
                    self.get_logger().info(
                        f'Published {self.measurement_count} measurements, '
                        f'{self.error_count} errors, '
                        f'latest: {distance:.3f}m'
                    )
            else:
                self.get_logger().warn(f'Distance out of range: {distance:.3f}m')
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KL200LaserNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()