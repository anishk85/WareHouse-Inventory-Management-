#!/usr/bin/env python3
"""
Magnetometer Publisher Node for ICM-20948
Reads magnetometer data and publishes to /imu/mag

This is a companion node to imu_publisher_node.py
Publishes magnetometer data as sensor_msgs/MagneticField

Install dependencies:
  sudo pip3 install adafruit-circuitpython-icm20x
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

try:
    import board
    import busio
    import adafruit_icm20x
    MAG_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Magnetometer libraries not available: {e}")
    print("Running in simulation mode")
    MAG_AVAILABLE = False


class MagnetometerPublisher(Node):
    def __init__(self):
        super().__init__('magnetometer_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 50.0)  # Hz (lower than IMU)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('simulate', not MAG_AVAILABLE)
        
        publish_rate = self.get_parameter('publish_rate').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.simulate = self.get_parameter('simulate').value
        
        # Publisher
        self.publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Initialize ICM-20948
        if not self.simulate and MAG_AVAILABLE:
            self.init_magnetometer()
        else:
            self.get_logger().warn('Running in SIMULATION mode')
            self.imu = None
        
        self.get_logger().info(f'Magnetometer Publisher started at {publish_rate} Hz')
    
    def init_magnetometer(self):
        """Initialize ICM-20948 magnetometer"""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu = adafruit_icm20x.ICM20948(i2c)
            
            # Configure magnetometer
            self.imu.magnetometer_data_rate = adafruit_icm20x.MagDataRate.RATE_100HZ
            
            self.get_logger().info('ICM-20948 magnetometer initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize magnetometer: {e}')
            self.get_logger().warn('Falling back to simulation mode')
            self.imu = None
            self.simulate = True
    
    def timer_callback(self):
        """Read magnetometer and publish data"""
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        
        if self.simulate or self.imu is None:
            # Fake data (approximate Earth's magnetic field)
            msg.magnetic_field.x = 20.0e-6  # Tesla
            msg.magnetic_field.y = 0.0e-6
            msg.magnetic_field.z = 40.0e-6
        else:
            try:
                # Read magnetometer (µT = micro-Tesla)
                mag = self.imu.magnetic
                
                # Convert µT to Tesla (SI unit)
                msg.magnetic_field.x = mag[0] * 1e-6
                msg.magnetic_field.y = mag[1] * 1e-6
                msg.magnetic_field.z = mag[2] * 1e-6
                
            except Exception as e:
                self.get_logger().error(f'Error reading magnetometer: {e}', 
                                       throttle_duration_sec=1.0)
                return
        
        # Set covariance (conservative estimate)
        # ICM-20948 magnetometer noise: ~0.15 µT
        mag_variance = (0.15e-6) ** 2  # Tesla²
        msg.magnetic_field_covariance = [
            mag_variance, 0.0, 0.0,
            0.0, mag_variance, 0.0,
            0.0, 0.0, mag_variance
        ]
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
