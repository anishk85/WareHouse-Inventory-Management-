#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>
#include <chrono>

#include "warehouse_rover_msgs/msg/qr_detection_array.hpp"
#include "warehouse_rover_msgs/msg/qr_detection.hpp"
#include "warehouse_rover_image_processing/hardware_detector.hpp"
#include "warehouse_rover_image_processing/image_preprocessor.hpp"
#include "warehouse_rover_qr_detection/qr_parser.hpp"

namespace warehouse_rover_image_processing
{

class QRDetectorEnhancedNode : public rclcpp::Node
{
public:
  explicit QRDetectorEnhancedNode(const rclcpp::NodeOptions & options)
  : Node("qr_detector_enhanced", options),
    frame_count_(0),
    detection_count_(0),
    total_processing_time_ms_(0)
  {
    // Detect hardware
    HardwareDetector hw_detector;
    hw_caps_ = hw_detector.detect();
    hw_detector.printCapabilities(hw_caps_);
    
    // Initialize preprocessor
    preprocessor_ = std::make_unique<ImagePreprocessor>(hw_caps_.recommended_mode);
    preprocessing_config_ = preprocessor_->getRecommendedConfig();
    
    // Parameters
    this->declare_parameter<bool>("enable_visualization", true);
    this->declare_parameter<bool>("enable_ipt", true);
    this->declare_parameter<bool>("enable_multipass", true);
    this->declare_parameter<int>("target_width", preprocessing_config_.target_width);
    this->declare_parameter<int>("target_height", preprocessing_config_.target_height);
    
    // Override config with parameters
    preprocessing_config_.enable_ipt = this->get_parameter("enable_ipt").as_bool();
    preprocessing_config_.target_width = this->get_parameter("target_width").as_int();
    preprocessing_config_.target_height = this->get_parameter("target_height").as_int();
    bool enable_viz = this->get_parameter("enable_visualization").as_bool();
    enable_multipass_ = this->get_parameter("enable_multipass").as_bool();
    
    // Initialize ZBar - ONLY QR CODES
    scanner_ = std::make_unique<zbar::ImageScanner>();
    scanner_->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);  // Disable all
    scanner_->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);  // Enable only QR
    
    // Subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      10,
      std::bind(&QRDetectorEnhancedNode::imageCallback, this, std::placeholders::_1)
    );
    
    // Publishers
    detection_pub_ = this->create_publisher<warehouse_rover_msgs::msg::QRDetectionArray>(
      "/qr_detections",
      10
    );
    
    if (enable_viz) {
      annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/qr_detections/image_annotated",
        10
      );
    }
    
    RCLCPP_INFO(this->get_logger(), "Enhanced QR Detector Node started");
    RCLCPP_INFO(this->get_logger(), "Processing Mode: %s", 
                getModeString(hw_caps_.recommended_mode).c_str());
    RCLCPP_INFO(this->get_logger(), "Target Resolution: %dx%d",
                preprocessing_config_.target_width,
                preprocessing_config_.target_height);
    RCLCPP_INFO(this->get_logger(), "IPT: %s, Multi-pass: %s",
                preprocessing_config_.enable_ipt ? "ON" : "OFF",
                enable_multipass_ ? "ON" : "OFF");
  }
  
  ~QRDetectorEnhancedNode()
  {
    if (frame_count_ > 0) {
      float avg_time = total_processing_time_ms_ / frame_count_;
      float detection_rate = 100.0f * detection_count_ / frame_count_;
      
      RCLCPP_INFO(this->get_logger(), "\n=== Performance Summary ===");
      RCLCPP_INFO(this->get_logger(), "Frames processed: %d", frame_count_);
      RCLCPP_INFO(this->get_logger(), "Detections: %d (%.1f%%)", detection_count_, detection_rate);
      RCLCPP_INFO(this->get_logger(), "Avg processing time: %.2f ms", avg_time);
      RCLCPP_INFO(this->get_logger(), "Effective FPS: %.1f", 1000.0f / avg_time);
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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
    
    warehouse_rover_msgs::msg::QRDetectionArray detections;
    
    if (enable_multipass_) {
      // Multi-pass strategy for maximum robustness
      detections = detectMultiPass(cv_ptr->image, msg->header);
    } else {
      // Single pass with preprocessing
      cv::Mat preprocessed = preprocessor_->process(cv_ptr->image, preprocessing_config_);
      detections = detectQRCodes(preprocessed, msg->header);
    }
    
    // Visualize if enabled
    if (annotated_pub_) {
      cv::Mat annotated = cv_ptr->image.clone();
      annotateImage(annotated, detections);
      auto annotated_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
      annotated_pub_->publish(*annotated_msg);
    }
    
    // Calculate total time
    auto end = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    detections.processing_time_ms = total_time;
    
    // Publish
    detection_pub_->publish(detections);
    
    // Statistics
    frame_count_++;
    total_processing_time_ms_ += total_time;
    
    if (detections.detection_count > 0) {
      detection_count_++;
      
      RCLCPP_INFO(
        this->get_logger(),
        "Frame %d: %d QR(s) | Total: %ldms",
        frame_count_,
        detections.detection_count,
        total_time
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
  
  warehouse_rover_msgs::msg::QRDetectionArray detectMultiPass(
    const cv::Mat & image,
    const std_msgs::msg::Header & header)
  {
    // Try multiple preprocessing strategies, return first success
    
    // Pass 1: Just grayscale + light blur (fastest, often works)
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);
    auto result = detectQRCodes(gray, header);
    if (result.detection_count > 0) return result;
    
    // Pass 2: CLAHE (better contrast)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    cv::Mat clahe_img;
    clahe->apply(gray, clahe_img);
    result = detectQRCodes(clahe_img, header);
    if (result.detection_count > 0) return result;
    
    // Pass 3: Histogram equalization
    cv::Mat equalized;
    cv::equalizeHist(gray, equalized);
    result = detectQRCodes(equalized, header);
    if (result.detection_count > 0) return result;
    
    // Pass 4: IPT if corners detected
    if (preprocessing_config_.enable_ipt) {
      std::vector<cv::Point2f> corners;
      if (preprocessor_->detectCorners(image, corners)) {
        cv::Mat warped = preprocessor_->applyIPT(image, corners);
        cv::cvtColor(warped, warped, cv::COLOR_BGR2GRAY);
        result = detectQRCodes(warped, header);
        if (result.detection_count > 0) return result;
      }
    }
    
    // No detection
    return result;
  }
  
  warehouse_rover_msgs::msg::QRDetectionArray detectQRCodes(
    const cv::Mat & image,
    const std_msgs::msg::Header & header)
  {
    warehouse_rover_msgs::msg::QRDetectionArray result;
    result.header = header;
    
    // Ensure grayscale
    cv::Mat gray;
    if (image.channels() == 3) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = image;
    }
    
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
      det.size_pixels = 100;
      
      result.detections.push_back(det);
    }
    
    result.detection_count = result.detections.size();
    return result;
  }
  
  void annotateImage(cv::Mat & image, const warehouse_rover_msgs::msg::QRDetectionArray & detections)
  {
    for (const auto & det : detections.detections) {
      cv::Rect bbox(det.center.x - 50, det.center.y - 50, 100, 100);
      
      if (det.is_valid) {
        cv::rectangle(image, bbox, cv::Scalar(0, 255, 0), 2);
        std::string label = det.rack_id + "_" + det.shelf_id + "_" + det.item_code;
        cv::putText(image, label, cv::Point(bbox.x, bbox.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
      } else {
        cv::rectangle(image, bbox, cv::Scalar(0, 0, 255), 2);
        cv::putText(image, "INVALID", cv::Point(bbox.x, bbox.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
      }
    }
  }
  
  std::string getModeString(ProcessingMode mode)
  {
    switch (mode) {
      case ProcessingMode::CUDA_AGGRESSIVE: return "CUDA Aggressive (RTX 4070)";
      case ProcessingMode::CUDA_OPTIMIZED: return "CUDA Optimized (Jetson)";
      case ProcessingMode::CUDA_BASIC: return "CUDA Basic";
      case ProcessingMode::CPU_OPTIMIZED: return "CPU Optimized";
      default: return "CPU Basic";
    }
  }
  
  // Hardware
  HardwareCapabilities hw_caps_;
  std::unique_ptr<ImagePreprocessor> preprocessor_;
  PreprocessingConfig preprocessing_config_;
  bool enable_multipass_;
  
  // ZBar
  std::unique_ptr<zbar::ImageScanner> scanner_;
  warehouse_rover_qr_detection::QRParser parser_;
  
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<warehouse_rover_msgs::msg::QRDetectionArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  
  // Stats
  int frame_count_;
  int detection_count_;
  long long total_processing_time_ms_;
};

}  // namespace warehouse_rover_image_processing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_rover_image_processing::QRDetectorEnhancedNode>(
    rclcpp::NodeOptions()
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
