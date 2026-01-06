#!/usr/bin/env python3
"""
Sensor Data Simulator for GUI Testing
Publishes mock sensor data to ROS2 topics so the GUI can display data
without real hardware being connected.
"""

import rclpy
from rclpy.node import Node
import math
import time
from datetime import datetime

from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion, Point, Vector3
from std_msgs.msg import Header


class SensorDataSimulator(Node):
    """Simulates sensor data for testing GUI"""
    
    def __init__(self):
        super().__init__('sensor_data_simulator')
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/mecanum_drive_controller/odometry', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.laser_distance_pub = self.create_publisher(Float32, '/laser/distance', 10)
        self.mission_status_pub = self.create_publisher(String, '/inventory_mission/status', 10)
        self.actuator_status_pub = self.create_publisher(String, '/actuator/status', 10)
        
        # Simulation parameters
        self.time_step = 0.1  # 100ms
        self.elapsed_time = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.heading = 0.0
        
        # Create timer for periodic publishing
        self.create_timer(self.time_step, self.publish_sensor_data)
        
        self.get_logger().info('🤖 Sensor Data Simulator Started')
        self.get_logger().info('Publishing mock data to:')
        self.get_logger().info('  - /imu/data')
        self.get_logger().info('  - /mecanum_drive_controller/odometry')
        self.get_logger().info('  - /scan')
        self.get_logger().info('  - /laser/distance')
        self.get_logger().info('  - /inventory_mission/status')
        self.get_logger().info('  - /actuator/status')
    
    def publish_sensor_data(self):
        """Publish all sensor data"""
        self.elapsed_time += self.time_step
        
        # Publish IMU data
        self.publish_imu()
        
        # Publish Odometry data
        self.publish_odometry()
        
        # Publish LiDAR data
        self.publish_lidar()
        
        # Publish laser distance
        self.publish_laser_distance()
        
        # Publish status messages
        self.publish_status_messages()
    
    def publish_imu(self):
        """Publish simulated IMU data"""
        msg = Imu()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='imu_link')
        
        # Simulate slow rotation
        angle = (self.elapsed_time * 0.1) % (2 * math.pi)
        
        # Quaternion from Euler angles (roll, pitch, yaw)
        q = self.euler_to_quaternion(
            roll=math.sin(self.elapsed_time * 0.05) * 0.1,
            pitch=math.cos(self.elapsed_time * 0.05) * 0.1,
            yaw=angle
        )
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Simulate acceleration
        msg.linear_acceleration = Vector3(
            x=math.sin(self.elapsed_time * 0.1) * 0.5,
            y=math.cos(self.elapsed_time * 0.1) * 0.3,
            z=9.81
        )
        
        # Simulate angular velocity
        msg.angular_velocity = Vector3(
            x=math.sin(self.elapsed_time * 0.15) * 0.1,
            y=math.cos(self.elapsed_time * 0.15) * 0.08,
            z=math.sin(self.elapsed_time * 0.2) * 0.2
        )
        
        self.imu_pub.publish(msg)
    
    def publish_odometry(self):
        """Publish simulated odometry data"""
        msg = Odometry()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')
        msg.child_frame_id = 'base_link'
        
        # Simulate moving in a circle
        self.position_x = math.sin(self.elapsed_time * 0.2) * 2.0
        self.position_y = math.cos(self.elapsed_time * 0.2) * 2.0
        self.position_z = math.sin(self.elapsed_time * 0.3) * 0.1
        self.heading = (self.elapsed_time * 0.2) % (2 * math.pi)
        
        # Pose
        q = self.euler_to_quaternion(0, 0, self.heading)
        msg.pose.pose = Pose(
            position=Point(x=self.position_x, y=self.position_y, z=self.position_z),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        
        # Twist (velocity)
        msg.twist.twist = Twist(
            linear=Vector3(
                x=math.cos(self.heading) * 0.5,
                y=math.sin(self.heading) * 0.5,
                z=0.0
            ),
            angular=Vector3(x=0.0, y=0.0, z=0.2)
        )
        
        self.odom_pub.publish(msg)
    
    def publish_lidar(self):
        """Publish simulated LiDAR scan data"""
        msg = LaserScan()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='lidar_link')
        
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.scan_time = 0.1
        
        # Simulate obstacle at different angles
        angles = []
        ranges = []
        num_points = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        
        for i in range(num_points):
            angle = msg.angle_min + i * msg.angle_increment
            angles.append(angle)
            
            # Simulate walls at certain distances
            base_range = 2.0
            
            # Add some obstacles
            if -0.3 < angle < 0.3:  # Front obstacle
                base_range = 0.8 + math.sin(self.elapsed_time * 0.5) * 0.3
            elif math.pi - 0.3 < abs(angle) < math.pi:  # Back
                base_range = 3.0
            elif -math.pi/2 - 0.3 < angle < -math.pi/2 + 0.3:  # Left wall
                base_range = 1.5
            elif math.pi/2 - 0.3 < angle < math.pi/2 + 0.3:  # Right wall
                base_range = 1.5
            
            # Add noise
            noise = (hash(str(angle)) % 100 / 1000) * 0.1
            range_val = base_range + noise
            range_val = max(msg.range_min, min(msg.range_max, range_val))
            ranges.append(float(range_val))
        
        msg.ranges = ranges
        self.lidar_pub.publish(msg)
    
    def publish_laser_distance(self):
        """Publish simulated front laser distance"""
        msg = Float32()
        # Distance to obstacle in front, oscillating
        msg.data = 1.0 + math.sin(self.elapsed_time * 0.5) * 0.5
        self.laser_distance_pub.publish(msg)
    
    def publish_status_messages(self):
        """Publish simulated status messages"""
        # Mission status
        mission_status = String()
        if int(self.elapsed_time / 5) % 3 == 0:
            mission_status.data = 'idle'
        elif int(self.elapsed_time / 5) % 3 == 1:
            mission_status.data = 'scanning_qr_codes'
        else:
            mission_status.data = 'scanning_complete'
        self.mission_status_pub.publish(mission_status)
        
        # Actuator status
        actuator_status = String()
        if int(self.elapsed_time / 3) % 2 == 0:
            actuator_status.data = 'idle'
        else:
            actuator_status.data = 'extending'
        self.actuator_status_pub.publish(actuator_status)
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[3] = cr * cp * cy + sr * sp * sy
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        
        return q


def main(args=None):
    rclpy.init(args=args)
    simulator = SensorDataSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        print('\n✋ Stopping simulator...')
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
