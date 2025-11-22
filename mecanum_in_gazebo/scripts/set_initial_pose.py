import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Give some time for subscribers (AMCL) to connect
        self.get_logger().info('Waiting for subscribers...')
        
        # Publish after a short delay to ensure connection
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        
        # header details
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # SET YOUR POSE HERE (x, y in meters, z, w for quaternion orientation)
        # Example: x=2.0, y=0.5, theta=0 (approx)
        msg.pose.pose.position.x = 2.0
        msg.pose.pose.position.y = 0.5
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # SET COVARIANCE (Important for AMCL to accept the pose)
        # Diagonal covariance: [x_var, y_var, z_var, roll_var, pitch_var, yaw_var]
        # A value of 0.25 is typical for "roughly known"
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Initial Pose: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}')
        
        # Stop the timer and exit
        self.timer.cancel()
        raise SystemExit # Clean exit to stop spinning

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()