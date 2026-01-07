#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera setup
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('publish_compressed', True)
        
        camera_idx = self.get_parameter('camera_index').value
        fps = self.get_parameter('fps').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.use_compressed = self.get_parameter('publish_compressed').value
        
        # Initialize camera with V4L2 backend for better compatibility
        self.cap = cv2.VideoCapture(camera_idx, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at index {camera_idx}!')
            self.get_logger().error('Try different camera_index values (0, 1, 10, 11, 12)')
            return
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Verify settings
        actual_width = int
        
        self.get_logger().info(f'Camera opened successfully: {width}x{height} @ {fps}fps')
        
        # Timer for publishing
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        # Publish raw image
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.image_pub.publish(msg)
            
            # Publish compressed image
            if self.use_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
                self.compressed_pub.publish(compressed_msg)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
