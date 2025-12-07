#!/usr/bin/env python3
# filepath: /root/ros2_ws/src/mecanum_hardware/scripts/laser_sensor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import gpiod
import time
import threading

class LaserSensorNode(Node):
    def __init__(self):
        super().__init__('laser_sensor_node')
        
        # GPIO pin configuration (BCM numbering)
        self.TRIGGER_PIN = 23  # Yellow wire - Input to sensor (trigger)
        self.ECHO_PIN = 17     # Blue wire - Output from sensor (echo)
        
        # Sensor specifications
        self.MAX_DISTANCE = 2.0  # 2 meters max range
        self.MIN_DISTANCE = 0.03  # 3 cm minimum range (typical for these sensors)
        self.SPEED_OF_LIGHT = 343.0  # m/s (speed of sound, used for timing)
        
        # Timeout for echo (if no response in 100ms, consider it out of range)
        self.TIMEOUT = 0.1  # 100ms timeout
        
        # Initialize GPIO using gpiod
        try:
            self.chip = gpiod.Chip('gpiochip0')
            
            # Get lines
            self.trigger_line = self.chip.get_line(self.TRIGGER_PIN)
            self.echo_line = self.chip.get_line(self.ECHO_PIN)
            
            # Configure trigger as output (starts LOW)
            self.trigger_line.request(
                consumer="laser_trigger",
                type=gpiod.LINE_REQ_DIR_OUT,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN,
                default_vals=[0]
            )
            
            # Configure echo as input with pull-down
            self.echo_line.request(
                consumer="laser_echo",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN
            )
            
            # Ensure trigger is LOW
            self.trigger_line.set_value(0)
            time.sleep(0.1)
            
            self.get_logger().info('GPIO initialized successfully')
            self.get_logger().info(f'Trigger Pin: GPIO{self.TRIGGER_PIN}')
            self.get_logger().info(f'Echo Pin: GPIO{self.ECHO_PIN}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            raise
        
        # Create publishers
        # Standard ROS Range message
        self.range_pub = self.create_publisher(Range, 'laser/range', 10)
        
        # Simple float distance in meters
        self.distance_pub = self.create_publisher(Float32, 'laser/distance', 10)
        
        # Measurement state
        self.current_distance = 0.0
        self.measurement_lock = threading.Lock()
        
        # Create timer for periodic measurements (10 Hz)
        self.measurement_timer = self.create_timer(0.1, self.measure_distance)
        
        self.get_logger().info('Laser Sensor Node initialized')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /laser/range (sensor_msgs/Range)')
        self.get_logger().info('  - /laser/distance (std_msgs/Float32)')
        self.get_logger().info('')
        self.get_logger().info('Sensor Range: 0.03m to 2.0m')
        self.get_logger().info('Measurement Rate: 10 Hz')
        
    def trigger_measurement(self):
        """Send trigger pulse to sensor (10us HIGH pulse)"""
        try:
            # Send 10 microsecond pulse
            self.trigger_line.set_value(0)
            time.sleep(0.000002)  # 2us LOW
            self.trigger_line.set_value(1)
            time.sleep(0.00001)   # 10us HIGH
            self.trigger_line.set_value(0)
        except Exception as e:
            self.get_logger().error(f'Trigger error: {e}')
    
    def wait_for_echo(self):
        """Wait for echo pulse and measure duration"""
        try:
            # Wait for echo to go HIGH (start of pulse)
            timeout_start = time.time()
            while self.echo_line.get_value() == 0:
                pulse_start = time.time()
                if (pulse_start - timeout_start) > self.TIMEOUT:
                    return -1  # Timeout waiting for pulse start
            
            # Wait for echo to go LOW (end of pulse)
            timeout_start = time.time()
            while self.echo_line.get_value() == 1:
                pulse_end = time.time()
                if (pulse_end - timeout_start) > self.TIMEOUT:
                    return -1  # Timeout waiting for pulse end
            
            # Calculate pulse duration
            pulse_duration = pulse_end - pulse_start
            
            # Convert to distance (time * speed / 2)
            # For laser sensors, timing might be different than ultrasonic
            # Adjust this formula based on sensor datasheet
            distance = (pulse_duration * self.SPEED_OF_LIGHT) / 2.0
            
            return distance
            
        except Exception as e:
            self.get_logger().error(f'Echo reading error: {e}')
            return -1
    
    def measure_distance(self):
        """Perform distance measurement and publish"""
        try:
            # Trigger measurement
            self.trigger_measurement()
            
            # Small delay before reading echo
            time.sleep(0.00001)  # 10us
            
            # Read distance
            distance = self.wait_for_echo()
            
            if distance > 0 and self.MIN_DISTANCE <= distance <= self.MAX_DISTANCE:
                with self.measurement_lock:
                    self.current_distance = distance
                
                # Publish Range message
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = 'laser_sensor_link'
                range_msg.radiation_type = Range.INFRARED  # Laser is infrared
                range_msg.field_of_view = 0.05  # ~3 degrees typical for laser sensors
                range_msg.min_range = float(self.MIN_DISTANCE)
                range_msg.max_range = float(self.MAX_DISTANCE)
                range_msg.range = float(distance)
                self.range_pub.publish(range_msg)
                
                # Publish simple distance
                distance_msg = Float32()
                distance_msg.data = float(distance)
                self.distance_pub.publish(distance_msg)
                
                self.get_logger().info(
                    f'Distance: {distance*100:.1f} cm ({distance:.3f} m)',
                    throttle_duration_sec=1.0  # Log once per second
                )
            elif distance > self.MAX_DISTANCE:
                self.get_logger().warn(
                    'Object out of range (>2m)',
                    throttle_duration_sec=5.0
                )
            elif distance > 0:  # Less than min distance
                self.get_logger().warn(
                    f'Object too close (<3cm)',
                    throttle_duration_sec=5.0
                )
            else:
                self.get_logger().warn(
                    'No echo received (timeout)',
                    throttle_duration_sec=5.0
                )
                
        except Exception as e:
            self.get_logger().error(f'Measurement error: {e}')
    
    def cleanup(self):
        """Cleanup GPIO on shutdown"""
        self.get_logger().info('Cleaning up GPIO...')
        try:
            self.trigger_line.set_value(0)
            time.sleep(0.01)
            
            self.trigger_line.release()
            self.echo_line.release()
            self.chip.close()
            
            self.get_logger().info('GPIO cleanup complete')
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = LaserSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt detected')
    except Exception as e:
        if node:
            node.get_logger().error(f'Error: {e}')
        else:
            print(f'Failed to initialize node: {e}')
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()