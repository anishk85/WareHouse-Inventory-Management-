#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')
        
        # QoS Profile matching EKF expectations (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ==========================================
        # 1. IMU FIXER
        # ==========================================
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/data', # Input (Raw from Gazebo)
            self.imu_callback, 
            10
        )
        self.imu_pub = self.create_publisher(
            Imu, 
            '/imu/data_fixed', # Output (To EKF)
            sensor_qos  # Use BEST_EFFORT to match EKF
        )
        
        # Define IMU covariances once
        self.imu_orientation_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.imu_angular_vel_cov = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

        # ==========================================
        # 2. ODOM FIXER (THE BRIDGE)
        # ==========================================
        # NOTE: We subscribe to the specific controller topic found in your list
        # self.odom_sub = self.create_subscription(
        #     Odometry, 
        #     '/mecanum_drive_controller/odometry', 
        #     self.odom_callback, 
        #     10
        # )
        
        # We publish to standard /odom so EKF can find it easily
        # CRITICAL: Use BEST_EFFORT QoS to match EKF expectations
        # self.odom_pub = self.create_publisher(
        #     Odometry, 
        #     '/odom', 
        #     sensor_qos  # Use BEST_EFFORT to match EKF
        # )

        # Define Odom covariance (High variance for X, Y, Yaw)
        # self.odom_cov = [
        #     0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        # ]

        self.get_logger().info('Sensor Bridge Started.')
        # self.get_logger().info('Bridging /mecanum_drive_controller/odometry -> /odom')
        self.get_logger().info('Fixing IMU Covariance on /imu/data_fixed')

    def imu_callback(self, msg):
        fixed_msg = Imu()
        fixed_msg.header = msg.header
        fixed_msg.orientation = msg.orientation
        fixed_msg.angular_velocity = msg.angular_velocity
        fixed_msg.linear_acceleration = msg.linear_acceleration
        
        # Apply Fixed Covariances
        fixed_msg.orientation_covariance = self.imu_orientation_cov
        fixed_msg.angular_velocity_covariance = self.imu_angular_vel_cov
        fixed_msg.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        
        self.imu_pub.publish(fixed_msg)

    # def odom_callback(self, msg):
    #     fixed_msg = Odometry()
    #     fixed_msg.header = msg.header
    #     fixed_msg.child_frame_id = msg.child_frame_id
    #     fixed_msg.pose = msg.pose
    #     fixed_msg.twist = msg.twist
        
    #     # Apply Fixed Covariances (Replace the 0.0s from Gazebo)
    #     fixed_msg.pose.covariance = self.odom_cov
    #     fixed_msg.twist.covariance = self.odom_cov
        
    #     self.odom_pub.publish(fixed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()