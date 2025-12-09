import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
from cv_bridge import CvBridge, CvBridgeError

class DepthPerceptionNode(Node):
    def __init__(self):
        super().__init__('depth_perception_node')

        # --- Parameters ---
        self.declare_parameter('video_topic', '/camera/image_raw')
        self.declare_parameter('stop_topic', '/lift/stop')
        self.declare_parameter('model_id', 'depth-anything/Depth-Anything-V2-Small-hf')
        self.declare_parameter('process_width', 480)
        self.declare_parameter('process_height', 360)
        self.declare_parameter('collision_threshold', 230)
        self.declare_parameter('roi_size', 300)
        self.declare_parameter('show_debug_windows', True)
        
        video_topic = self.get_parameter('video_topic').get_parameter_value().string_value
        stop_topic = self.get_parameter('stop_topic').get_parameter_value().string_value
        model_id = self.get_parameter('model_id').get_parameter_value().string_value
        self.process_width = self.get_parameter('process_width').get_parameter_value().integer_value
        self.process_height = self.get_parameter('process_height').get_parameter_value().integer_value
        self.COLLISION_THRESHOLD = self.get_parameter('collision_threshold').get_parameter_value().integer_value
        self.roi_size = self.get_parameter('roi_size').get_parameter_value().integer_value
        self.show_debug_windows = self.get_parameter('show_debug_windows').get_parameter_value().bool_value

        # --- Class Members ---
        self.bridge = CvBridge()
        
        # --- ROS2 Publishers and Subscribers ---
        self.image_sub = self.create_subscription(
            Image, video_topic, self.image_callback, 10
        )
        self.stop_pub = self.create_publisher(Bool, stop_topic, 10)

        self.get_logger().info(f"Loading model '{model_id}'...")
        try:
            self.processor = AutoImageProcessor.from_pretrained(model_id)
            self.model = AutoModelForDepthEstimation.from_pretrained(model_id)
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            self.model.to(self.device)
            self.quantized_model = torch.quantization.quantize_dynamic(
                                        self.model, 
                                        {torch.nn.Linear},  # Layers to quantize
                                        dtype=torch.qint8   # Target data type
                                    )
            self.get_logger().info(f"Model loaded successfully on {self.device}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.destroy_node()
            return
            
        self.get_logger().info(f"Node initialized. Subscribed to {video_topic}")
        if self.show_debug_windows:
            self.get_logger().info("Debug windows are enabled.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return
        
        small_frame = cv2.resize(frame, (self.process_width, self.process_height))
        
        inputs = self.processor(images=small_frame, return_tensors="pt").to(self.device)
        
        with torch.no_grad():
            outputs = self.quantized_model(**inputs)
            predicted_depth = outputs.predicted_depth

        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=small_frame.shape[:2],
            mode="bicubic",
            align_corners=False,
        )
        
        depth_output = prediction.squeeze().cpu().numpy()
        
        depth_min = depth_output.min()
        depth_max = depth_output.max()
        
        if depth_max - depth_min > 0:
            depth_uint8 = (255 * (depth_output - depth_min) / (depth_max - depth_min)).astype("uint8")
        else:
            depth_uint8 = np.zeros_like(depth_output, dtype="uint8")

        h, w = depth_uint8.shape
        center_y, center_x = h // 2, w // 2
        
        y1 = max(0, center_y - self.roi_size//2)
        y2 = min(h, center_y + self.roi_size//2)
        x1 = max(0, center_x - self.roi_size//2)
        x2 = min(w, center_x + self.roi_size//2)
        
        roi = depth_uint8[y1:y2, x1:x2]
        
        avg_closeness = 0
        if roi.size > 0:
            avg_closeness = np.percentile(roi, 95)

        stop_signal = avg_closeness > self.COLLISION_THRESHOLD
        
        stop_msg = Bool()
        stop_msg.data = bool(stop_signal)
        self.stop_pub.publish(stop_msg)

        if self.show_debug_windows:
            color_depth = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_MAGMA)
            
            status = "PATH CLEAR"
            status_color = (0, 255, 0)
            
            if stop_signal:
                status = "CRITICAL STOP!"
                status_color = (0, 0, 255)

            cv2.rectangle(color_depth, (x1, y1), (x2, y2), (255, 255, 255), 2)
            cv2.putText(color_depth, f"{status}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv2.putText(color_depth, f"Score: {int(avg_closeness)}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            cv2.imshow("Depth Perception", color_depth)
            
            if cv2.waitKey(1) == ord('q'):
                self.get_logger().info("'q' pressed, shutting down.")
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DepthPerceptionNode()
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
