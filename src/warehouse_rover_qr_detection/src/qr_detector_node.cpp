#include "warehouse_rover_qr_detection/qr_detector_node.hpp"
#include <chrono>

namespace warehouse_rover_qr_detection
{

QRDetectorNode::QRDetectorNode(const rclcpp::NodeOptions & options)
: Node("qr_detector_node", options),
  frame_count_(0),
  detection_count_(0)
{
  // Initialize ZBar
  scanner_ = std::make_unique<zbar::ImageScanner>();
  scanner_->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  
  // Subscribers
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw",
    10,
    std::bind(&QRDetectorNode::imageCallback, this, std::placeholders::_1)
  );
  
  // Publishers
  detection_pub_ = this->create_publisher<warehouse_rover_msgs::msg::QRDetectionArray>(
    "/qr_detections",
    10
  );
  
  RCLCPP_INFO(this->get_logger(), "QR Detector Node started");
}

void QRDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto start = std::chrono::high_resolution_clock::now();
  
  // Convert to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
  }
  
  // Detect QR codes
  auto detections = detectQRCodes(cv_ptr->image, msg->header);
  
  // Calculate processing time
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  detections.processing_time_ms = duration.count();
  
  // Publish
  detection_pub_->publish(detections);
  
  // Log
  frame_count_++;
  if (detections.detection_count > 0) {
    detection_count_++;
    
    RCLCPP_INFO(
      this->get_logger(),
      "Frame %d: Detected %d QR code(s) in %ld ms",
      frame_count_,
      detections.detection_count,
      duration.count()
    );
    
    for (const auto & det : detections.detections) {
      if (det.is_valid) {
        RCLCPP_INFO(
          this->get_logger(),
          "  ✓ [%s][%s] %s (conf: %.2f)",
          det.rack_id.c_str(),
          det.shelf_id.c_str(),
          det.item_code.c_str(),
          det.confidence
        );
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "  ✗ Invalid: \"%s\" - %s",
          det.qr_data.c_str(),
          det.error_message.c_str()
        );
      }
    }
  }
}

warehouse_rover_msgs::msg::QRDetectionArray QRDetectorNode::detectQRCodes(
  const cv::Mat & image,
  const std_msgs::msg::Header & header)
{
  warehouse_rover_msgs::msg::QRDetectionArray result;
  result.header = header;
  
  // Convert to grayscale
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image.clone();
  }
  
  // Preprocessing
  cv::equalizeHist(gray, gray);
  
  // Scan with ZBar
  zbar::Image zbar_img(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
  scanner_->scan(zbar_img);
  
  // Process detections
  for (auto it = zbar_img.symbol_begin(); it != zbar_img.symbol_end(); ++it) {
    warehouse_rover_msgs::msg::QRDetection det;
    
    // Raw data
    det.qr_data = it->get_data();
    
    // Parse
    auto parsed = parser_.parse(det.qr_data);
    det.rack_id = parsed.rack_id;
    det.shelf_id = parsed.shelf_id;
    det.item_code = parsed.item_code;
    det.is_valid = parsed.is_valid;
    det.error_message = parsed.error_message;
    
    // Metadata
    det.confidence = it->get_quality() / 255.0f;
    
    // Calculate center
    int sum_x = 0, sum_y = 0;
    int count = it->get_location_size();
    for (int i = 0; i < count; ++i) {
      sum_x += it->get_location_x(i);
      sum_y += it->get_location_y(i);
    }
    det.center.x = count > 0 ? sum_x / count : 0;
    det.center.y = count > 0 ? sum_y / count : 0;
    det.size_pixels = 100; // Approximate
    
    result.detections.push_back(det);
  }
  
  result.detection_count = result.detections.size();
  return result;
}

}  // namespace warehouse_rover_qr_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_rover_qr_detection::QRDetectorNode>(
    rclcpp::NodeOptions()
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
