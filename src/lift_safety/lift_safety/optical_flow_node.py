import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class OpticalFlowSafetyNode(Node):
    def __init__(self):
        super().__init__('optical_flow_safety_node')

        # --- Parameters ---
        self.declare_parameter('video_topic', '/camera/image_raw')
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('magnitude_threshold', 12.0)
        self.declare_parameter('min_area', 800)
        self.declare_parameter('angle_tolerance', 25)
        self.declare_parameter('show_debug_windows', True)

        video_topic = self.get_parameter('video_topic').get_parameter_value().string_value
        self.FRAME_WIDTH = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.MAGNITUDE_THRESHOLD = self.get_parameter('magnitude_threshold').get_parameter_value().double_value
        self.MIN_AREA = self.get_parameter('min_area').get_parameter_value().integer_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().integer_value
        self.show_debug_windows = self.get_parameter('show_debug_windows').get_parameter_value().bool_value
        stop_topic = self.get_parameter('stop_topic').get_parameter_value().string_value

        # --- Class Members ---
        self.bridge = CvBridge()
        self.prvs = None
        self.hsv = None

        # --- ROS2 Publishers and Subscribers ---
        self.image_sub = self.create_subscription(
            Image, video_topic, self.image_callback, 10
        )
        self.collision_pub = self.create_publisher(Bool, stop_topic, 10)

        self.get_logger().info(f"Node initialized. Subscribed to {video_topic}")
        self.get_logger().info("Press 'q' in the OpenCV window to quit.")
        if self.show_debug_windows:
            self.get_logger().info("Debug windows are enabled.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # --- Initialization of buffers on first frame ---
        if self.prvs is None:
            self.prvs = self._prepare_frame(frame)
            self.hsv = np.zeros_like(frame)
            self.hsv[..., 1] = 255  # Max saturation
            return

        # --- Main Optical Flow Logic ---
        frame_resized = cv2.resize(frame, (self.FRAME_WIDTH, int(frame.shape[0] * (self.FRAME_WIDTH / frame.shape[1]))))
        next_frame_gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

        # 1. Calculate Dense Optical Flow
        flow = cv2.calcOpticalFlowFarneback(self.prvs, next_frame_gray, None,
                                            0.5, 3, 15, 3, 5, 1.2, 0)

        # 2. Convert Flow to Polar coordinates
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1], angleInDegrees=True)

        # 3. Create the "Alarm Mask"
        motion_mask = mag > self.MAGNITUDE_THRESHOLD
        
        ignore_up = (ang > (90 - self.angle_tolerance)) & (ang < (90 + self.angle_tolerance))
        ignore_down = (ang > (270 - self.angle_tolerance)) & (ang < (270 + self.angle_tolerance))
        ignore_mask = ignore_up | ignore_down
        
        alarm_mask = motion_mask & (~ignore_mask)
        
        alarm_mask_uint8 = alarm_mask.astype(np.uint8) * 255
        kernel = np.ones((3, 3), np.uint8)
        alarm_mask_uint8 = cv2.morphologyEx(alarm_mask_uint8, cv2.MORPH_OPEN, kernel)

        # 4. Find Contours and Detect Intrusion
        contours, _ = cv2.findContours(alarm_mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        intrusion_detected = False
        for contour in contours:
            if cv2.contourArea(contour) > self.MIN_AREA:
                intrusion_detected = True
                if self.show_debug_windows:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame_resized, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame_resized, "INTRUSION", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # --- Publish Collision Status ---
        collision_msg = Bool()
        collision_msg.data = intrusion_detected
        self.collision_pub.publish(collision_msg)

        # --- Visualization ---
        if self.show_debug_windows:
            status_text = "Status: SAFE"
            color = (0, 255, 0)
            if intrusion_detected:
                status_text = "Status: WARNING - OBJ DETECTED"
                color = (0, 0, 255)
            
            cv2.putText(frame_resized, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.imshow('Lift Feed', frame_resized)
            cv2.imshow('Debug: Alarm Mask', alarm_mask_uint8)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("'q' pressed, shutting down.")
                self.destroy_node()
                rclpy.shutdown()
        
        # --- Update Previous Frame ---
        self.prvs = next_frame_gray

    def _prepare_frame(self, frame):
        """Resizes and converts a frame to grayscale."""
        frame_resized = cv2.resize(frame, (self.FRAME_WIDTH, int(frame.shape[0] * (self.FRAME_WIDTH / frame.shape[1]))))
        return cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
