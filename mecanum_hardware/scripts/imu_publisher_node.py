#!/usr/bin/env python3
"""
IMU Publisher Node for Raspberry Pi 5
Reads IMU data from GPIO and publishes to /imu/data_raw

Supports common IMU modules:
- MPU6050 (I2C)
- MPU9250 (I2C)
- BNO055 (I2C/UART)
- LSM9DS1 (I2C/SPI)

Install dependencies:
  sudo apt-get install python3-smbus python3-dev i2c-tools
  sudo pip3 install adafruit-circuitpython-mpu6050
  # OR for BNO055:
  sudo pip3 install adafruit-circuitpython-bno055
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math

# Choose your IMU type here
IMU_TYPE = "MPU6050"  # Options: "MPU6050", "BNO055", "MPU9250"

try:
    import board
    import busio
    
    if IMU_TYPE == "MPU6050":
        import adafruit_mpu6050
    elif IMU_TYPE == "BNO055":
        import adafruit_bno055
    elif IMU_TYPE == "MPU9250":
        # Add MPU9250 library import if needed
        pass
    
    IMU_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: IMU libraries not available: {e}")
    print("Running in simulation mode - publishing fake IMU data")
    IMU_AVAILABLE = False


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('simulate', not IMU_AVAILABLE)
        
        publish_rate = self.get_parameter('publish_rate').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.simulate = self.get_parameter('simulate').value
        
        # Publisher
        self.publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Initialize IMU hardware
        if not self.simulate and IMU_AVAILABLE:
            self.init_imu()
        else:
            self.get_logger().warn('Running in SIMULATION mode - publishing fake IMU data')
            self.imu = None
        
        self.get_logger().info(f'IMU Publisher started at {publish_rate} Hz')
    
    def init_imu(self):
        """Initialize IMU hardware"""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            
            if IMU_TYPE == "MPU6050":
                self.imu = adafruit_mpu6050.MPU6050(i2c)
                self.get_logger().info('MPU6050 initialized successfully')
                
                # Configure MPU6050
                self.imu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
                self.imu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS
                self.imu.filter_bandwidth = adafruit_mpu6050.Bandwidth.BAND_21_HZ
                
            elif IMU_TYPE == "BNO055":
                self.imu = adafruit_bno055.BNO055_I2C(i2c)
                self.get_logger().info('BNO055 initialized successfully')
                
            else:
                self.get_logger().error(f'Unsupported IMU type: {IMU_TYPE}')
                self.imu = None
                self.simulate = True
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize IMU: {e}')
            self.get_logger().warn('Falling back to simulation mode')
            self.imu = None
            self.simulate = True
    
    def timer_callback(self):
        """Read IMU and publish data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        
        if self.simulate or self.imu is None:
            # Fake data for testing without hardware
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 9.81  # Gravity
            
            # No orientation available without magnetometer
            msg.orientation_covariance[0] = -1.0  # -1 means no orientation data
            
        else:
            try:
                if IMU_TYPE == "MPU6050":
                    # Read gyroscope (rad/s)
                    gyro = self.imu.gyro
                    msg.angular_velocity.x = gyro[0]
                    msg.angular_velocity.y = gyro[1]
                    msg.angular_velocity.z = gyro[2]
                    
                    # Read accelerometer (m/s²)
                    accel = self.imu.acceleration
                    msg.linear_acceleration.x = accel[0]
                    msg.linear_acceleration.y = accel[1]
                    msg.linear_acceleration.z = accel[2]
                    
                    # MPU6050 doesn't have magnetometer - no orientation
                    msg.orientation_covariance[0] = -1.0
                    
                elif IMU_TYPE == "BNO055":
                    # BNO055 provides quaternion orientation
                    quat = self.imu.quaternion
                    if quat is not None:
                        msg.orientation.w = quat[0]
                        msg.orientation.x = quat[1]
                        msg.orientation.y = quat[2]
                        msg.orientation.z = quat[3]
                    else:
                        msg.orientation_covariance[0] = -1.0
                    
                    # Read gyroscope
                    gyro = self.imu.gyro
                    if gyro is not None:
                        msg.angular_velocity.x = math.radians(gyro[0])
                        msg.angular_velocity.y = math.radians(gyro[1])
                        msg.angular_velocity.z = math.radians(gyro[2])
                    
                    # Read accelerometer
                    accel = self.imu.acceleration
                    if accel is not None:
                        msg.linear_acceleration.x = accel[0]
                        msg.linear_acceleration.y = accel[1]
                        msg.linear_acceleration.z = accel[2]
                        
            except Exception as e:
                self.get_logger().error(f'Error reading IMU: {e}', throttle_duration_sec=1.0)
                return
        
        # Set covariances (conservative estimates)
        # These should be tuned based on your actual IMU specifications
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
        
        if msg.orientation_covariance[0] != -1.0:
            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
