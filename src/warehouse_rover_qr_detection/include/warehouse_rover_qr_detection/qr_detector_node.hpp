#ifndef WAREHOUSE_ROVER_QR_DETECTION__QR_DETECTOR_NODE_HPP_
#define WAREHOUSE_ROVER_QR_DETECTION__QR_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

#include "warehouse_rover_msgs/msg/qr_detection_array.hpp"
#include "warehouse_rover_msgs/msg/qr_detection.hpp"
#include "warehouse_rover_qr_detection/qr_parser.hpp"

namespace warehouse_rover_qr_detection
{

class QRDetectorNode : public rclcpp::Node
{
public:
  explicit QRDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  warehouse_rover_msgs::msg::QRDetectionArray detectQRCodes(
    const cv::Mat & image,
    const std_msgs::msg::Header & header
  );
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<warehouse_rover_msgs::msg::QRDetectionArray>::SharedPtr detection_pub_;
  
  std::unique_ptr<zbar::ImageScanner> scanner_;
  QRParser parser_;
  
  int frame_count_;
  int detection_count_;
};

}  // namespace warehouse_rover_qr_detection

#endif
