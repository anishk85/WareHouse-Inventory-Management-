#!/usr/bin/env python3
"""
IMU Publisher Node for Raspberry Pi 5 (Docker Compatible)
Reads IMU data from I2C and publishes to /imu/data_raw

Supports:
- ICM-20948 (I2C) - 9-DOF with magnetometer ✓ RECOMMENDED

Docker Setup:
  docker run --device=/dev/i2c-1 --privileged ...
  
Install dependencies in container:
  sudo apt-get install -y i2c-tools python3-smbus
  sudo pip3 install smbus2 adafruit-blinka adafruit-circuitpython-icm20x
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import math
import os

# I2C Configuration
I2C_BUS = 1  # Standard I2C bus on Raspberry Pi
ICM20948_ADDR = 0x69  # Default ICM-20948 address

# Check if running in Docker
IN_DOCKER = os.path.exists('/.dockerenv')

try:
    # Try direct I2C access first (works in Docker)
    from smbus2 import SMBus
    SMBUS_AVAILABLE = True
    
    # Also try to import Adafruit libraries
    try:
        import board
        import busio
        import adafruit_icm20x
        ADAFRUIT_AVAILABLE = True
    except ImportError:
        ADAFRUIT_AVAILABLE = False
        print("WARNING: Adafruit libraries not available, using direct I2C")
    
except ImportError as e:
    print(f"WARNING: I2C libraries not available: {e}")
    print("Running in simulation mode")
    SMBUS_AVAILABLE = False
    ADAFRUIT_AVAILABLE = False


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('i2c_bus', I2C_BUS)
        self.declare_parameter('simulate', False)
        
        publish_rate = self.get_parameter('publish_rate').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.simulate = self.get_parameter('simulate').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Initialize IMU hardware
        self.imu = None
        self.use_adafruit = False
        
        if not self.simulate:
            if IN_DOCKER:
                self.get_logger().info('Running in Docker container')
            
            if not self.init_imu():
                self.get_logger().warn('Failed to initialize IMU, running in simulation mode')
                self.simulate = True
        else:
            self.get_logger().warn('Running in SIMULATION mode')
        
        self.get_logger().info(f'IMU Publisher started at {publish_rate} Hz')
    
    def init_imu(self):
        """Initialize IMU hardware - try Adafruit first, fall back to SMBus"""
        
        # Try Adafruit library first (easier, higher level)
        if ADAFRUIT_AVAILABLE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.imu = adafruit_icm20x.ICM20948(i2c)
                
                # Configure ICM-20948
                self.imu.accelerometer_range = adafruit_icm20x.AccelRange.RANGE_2G
                self.imu.gyro_range = adafruit_icm20x.GyroRange.RANGE_250_DPS
                self.imu.magnetometer_data_rate = adafruit_icm20x.MagDataRate.RATE_100HZ
                
                self.use_adafruit = True
                self.get_logger().info('ICM-20948 initialized via Adafruit library')
                return True
                
            except Exception as e:
                self.get_logger().warn(f'Adafruit library init failed: {e}')
        
        # Fall back to direct SMBus access (works better in Docker)
        if SMBUS_AVAILABLE:
            try:
                self.bus = SMBus(self.i2c_bus)
                
                # Test connection by reading WHO_AM_I register
                who_am_i = self.bus.read_byte_data(ICM20948_ADDR, 0x00)
                
                if who_am_i == 0xEA:
                    # Wake up device
                    self.bus.write_byte_data(ICM20948_ADDR, 0x06, 0x01)
                    
                    self.use_adafruit = False
                    self.get_logger().info(f'ICM-20948 initialized via SMBus (WHO_AM_I=0x{who_am_i:02X})')
                    return True
                else:
                    self.get_logger().error(f'Wrong WHO_AM_I: 0x{who_am_i:02X} (expected 0xEA)')
                    return False
                    
            except Exception as e:
                self.get_logger().error(f'SMBus init failed: {e}')
                self.get_logger().info('Make sure Docker has I2C access: --device=/dev/i2c-1 --privileged')
                return False
        
        return False
    
    def read_word(self, reg):
        """Read 16-bit signed word from ICM-20948"""
        high = self.bus.read_byte_data(ICM20948_ADDR, reg)
        low = self.bus.read_byte_data(ICM20948_ADDR, reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    
    def read_imu_smbus(self):
        """Read IMU data using direct SMBus access"""
        # Register addresses
        ACCEL_XOUT_H = 0x2D
        GYRO_XOUT_H = 0x33
        
        # Read accelerometer (±2g = 16384 LSB/g)
        accel_x = self.read_word(ACCEL_XOUT_H) / 16384.0 * 9.81
        accel_y = self.read_word(ACCEL_XOUT_H + 2) / 16384.0 * 9.81
        accel_z = self.read_word(ACCEL_XOUT_H + 4) / 16384.0 * 9.81
        
        # Read gyroscope (±250°/s = 131 LSB/(°/s))
        gyro_x = math.radians(self.read_word(GYRO_XOUT_H) / 131.0)
        gyro_y = math.radians(self.read_word(GYRO_XOUT_H + 2) / 131.0)
        gyro_z = math.radians(self.read_word(GYRO_XOUT_H + 4) / 131.0)
        
        return {
            'accel': (accel_x, accel_y, accel_z),
            'gyro': (gyro_x, gyro_y, gyro_z),
            'mag': None  # Magnetometer requires bank switching - not implemented yet
        }
    
    def timer_callback(self):
        """Read IMU and publish data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        
        if self.simulate or self.imu is None:
            # Fake data for testing
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 9.81  # Gravity
            msg.orientation_covariance[0] = -1.0
            
        else:
            try:
                if self.use_adafruit:
                    # Read using Adafruit library
                    gyro = self.imu.gyro
                    accel = self.imu.acceleration
                    mag = self.imu.magnetic
                    
                    msg.angular_velocity.x = gyro[0]
                    msg.angular_velocity.y = gyro[1]
                    msg.angular_velocity.z = gyro[2]
                    
                    msg.linear_acceleration.x = accel[0]
                    msg.linear_acceleration.y = accel[1]
                    msg.linear_acceleration.z = accel[2]
                    
                    # Publish magnetometer separately
                    if mag is not None:
                        mag_msg = MagneticField()
                        mag_msg.header = msg.header
                        mag_msg.magnetic_field.x = mag[0] * 1e-6  # µT to T
                        mag_msg.magnetic_field.y = mag[1] * 1e-6
                        mag_msg.magnetic_field.z = mag[2] * 1e-6
                        self.mag_pub.publish(mag_msg)
                    
                else:
                    # Read using direct SMBus
                    data = self.read_imu_smbus()
                    
                    gyro = data['gyro']
                    msg.angular_velocity.x = gyro[0]
                    msg.angular_velocity.y = gyro[1]
                    msg.angular_velocity.z = gyro[2]
                    
                    accel = data['accel']
                    msg.linear_acceleration.x = accel[0]
                    msg.linear_acceleration.y = accel[1]
                    msg.linear_acceleration.z = accel[2]
                
                # No orientation from raw IMU
                msg.orientation_covariance[0] = -1.0
                
            except Exception as e:
                self.get_logger().error(f'Error reading IMU: {e}', throttle_duration_sec=1.0)
                return
        
        # Set covariances
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()