#include "warehouse_rover_rack_detection/rack_detector_node.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>

namespace warehouse_rover_rack_detection
{

RackDetectorNode::RackDetectorNode(const rclcpp::NodeOptions & options)
: Node("rack_detector", options),
  first_detection_(true)
{
  // Detection parameters
  this->declare_parameter<int>("threshold", 50);
  this->declare_parameter<int>("min_area", 5);
  this->declare_parameter<int>("max_area", 500);
  this->declare_parameter<double>("min_aspect_ratio", 0.2);
  this->declare_parameter<double>("max_aspect_ratio", 5.0);
  this->declare_parameter<double>("waypoint_offset", 0.6);
  
  // Rack dimensions
  this->declare_parameter<double>("rack_length", 0.91);
  this->declare_parameter<double>("rack_width", 0.35);
  this->declare_parameter<double>("rack_height", 1.0);
  this->declare_parameter<std::vector<double>>("shelf_heights", 
    std::vector<double>{0.25, 0.50, 0.75, 1.0});
  
  // Get parameters
  this->get_parameter("threshold", threshold_);
  this->get_parameter("min_area", min_area_);
  this->get_parameter("max_area", max_area_);
  this->get_parameter("min_aspect_ratio", min_aspect_ratio_);
  this->get_parameter("max_aspect_ratio", max_aspect_ratio_);
  this->get_parameter("waypoint_offset", waypoint_offset_);
  
  this->get_parameter("rack_length", rack_length_);
  this->get_parameter("rack_width", rack_width_);
  this->get_parameter("rack_height", rack_height_);
  this->get_parameter("shelf_heights", shelf_heights_);
  
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).transient_local(),
    std::bind(&RackDetectorNode::mapCallback, this, std::placeholders::_1)
  );
  
  // Publishers
  rack_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detected_racks",
    10
  );
  
  waypoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/rack_waypoints",
    10
  );
  
  RCLCPP_INFO(this->get_logger(), "Rack Detector Node started");
  RCLCPP_INFO(this->get_logger(), "Waiting for /map topic...");
}

void RackDetectorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (first_detection_) {
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "═══ MAP INFO ═══");
    RCLCPP_INFO(this->get_logger(), "Size: %dx%d", msg->info.width, msg->info.height);
    RCLCPP_INFO(this->get_logger(), "Resolution: %.3fm/cell", msg->info.resolution);
    RCLCPP_INFO(this->get_logger(), "Origin: (%.2f, %.2f)", 
                msg->info.origin.position.x, msg->info.origin.position.y);
    first_detection_ = false;
  }
  
  // Detect racks
  detected_racks_ = detectRacks(*msg);
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "═══ DETECTED %zu RACKS ═══", detected_racks_.size());
  
  // Publish visualization
  publishMarkers(detected_racks_, msg->header);
  
  // Log positions
  for (size_t i = 0; i < detected_racks_.size(); ++i) {
    const auto & rack = detected_racks_[i];
    RCLCPP_INFO(
      this->get_logger(),
      "  Rack %zu: (%.2f, %.2f) θ=%.1f°",
      i + 1, rack.x, rack.y, rack.theta * 180.0 / M_PI
    );
  }
}

std::vector<RackPose> RackDetectorNode::detectRacks(
  const nav_msgs::msg::OccupancyGrid & map_msg)
{
  std::vector<RackPose> racks;
  
  int width = map_msg.info.width;
  int height = map_msg.info.height;
  double resolution = map_msg.info.resolution;
  
  // Step 1: Convert to binary
  cv::Mat binary(height, width, CV_8UC1);
  for (int i = 0; i < height * width; ++i) {
    int8_t value = map_msg.data[i];
    binary.data[i] = (value >= threshold_) ? 255 : 0;
  }
  
  // Step 2: Remove walls - use smaller erosion
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
  cv::Mat processed;
  cv::erode(binary, processed, kernel);
  
  // Step 3: Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(processed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  RCLCPP_INFO(this->get_logger(), "Found %zu total contours", contours.size());
  
  // Step 4: Collect valid blob centers with debugging
  std::vector<cv::Point2f> blob_centers;
  int valid_count = 0;
  
  for (const auto & contour : contours) {
    double area = cv::contourArea(contour);
    
    if (area >= min_area_ && area <= max_area_) {
      cv::Moments M = cv::moments(contour);
      if (M.m00 > 0) {
        float cx = M.m10 / M.m00;
        float cy = M.m01 / M.m00;
        blob_centers.push_back(cv::Point2f(cx, cy));
        valid_count++;
        
        // Debug: print first few blobs
        if (valid_count <= 10) {
          double wx = map_msg.info.origin.position.x + cx * resolution;
          double wy = map_msg.info.origin.position.y + (height - cy) * resolution;
          RCLCPP_DEBUG(this->get_logger(), 
            "  Blob %d: pixel(%.0f,%.0f) → world(%.2f,%.2f) area=%.0f",
            valid_count, cx, cy, wx, wy, area);
        }
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Valid blobs: %d (area %d-%d px²)", 
              valid_count, min_area_, max_area_);
  
  // Step 5: Cluster blobs
  std::vector<std::vector<cv::Point2f>> clusters;
  std::vector<bool> visited(blob_centers.size(), false);
  
  // Cluster distance based on rack size
  double cluster_dist_m = 0.8;  // 80cm
  double cluster_dist_px = cluster_dist_m / resolution;
  
  RCLCPP_INFO(this->get_logger(), "Clustering with distance %.0fpx (%.2fm)", 
              cluster_dist_px, cluster_dist_m);
  
  for (size_t i = 0; i < blob_centers.size(); ++i) {
    if (visited[i]) continue;
    
    std::vector<cv::Point2f> cluster;
    cluster.push_back(blob_centers[i]);
    visited[i] = true;
    
    for (size_t j = i + 1; j < blob_centers.size(); ++j) {
      if (visited[j]) continue;
      
      double dist = cv::norm(blob_centers[i] - blob_centers[j]);
      if (dist < cluster_dist_px) {
        cluster.push_back(blob_centers[j]);
        visited[j] = true;
      }
    }
    
    if (cluster.size() >= 1 && cluster.size() <= 6) {
      clusters.push_back(cluster);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Formed %zu clusters", clusters.size());
  
  // Step 6: Convert to racks
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto & cluster = clusters[i];
    
    // Calculate center
    float sum_x = 0, sum_y = 0;
    for (const auto & pt : cluster) {
      sum_x += pt.x;
      sum_y += pt.y;
    }
    float cx = sum_x / cluster.size();
    float cy = sum_y / cluster.size();
    
    // Convert to world coordinates
    // IMPORTANT: Y axis is flipped in image coordinates
    double world_x = map_msg.info.origin.position.x + cx * resolution;
    double world_y = map_msg.info.origin.position.y + (height - cy) * resolution;
    
    RCLCPP_INFO(this->get_logger(), 
      "Cluster %zu: %zu blobs, pixel(%.0f,%.0f) → world(%.2f,%.2f)",
      i + 1, cluster.size(), cx, cy, world_x, world_y);
    
    // Estimate orientation
    double theta = 0.0;
    if (cluster.size() >= 2) {
      cv::Point2f diff = cluster[1] - cluster[0];
      theta = std::atan2(diff.y, diff.x);
    }
    
    RackPose rack;
    rack.x = world_x;
    rack.y = world_y;
    rack.theta = theta;
    rack.width = rack_length_;
    rack.depth = rack_width_;
    
    racks.push_back(rack);
  }
  
  // Save debug image
  cv::Mat debug_img = binary.clone();
  cv::cvtColor(debug_img, debug_img, cv::COLOR_GRAY2BGR);
  
  // Draw all detected blobs in blue
  for (const auto & center : blob_centers) {
    cv::circle(debug_img, cv::Point(center.x, center.y), 3, cv::Scalar(255, 0, 0), -1);
  }
  
  // Draw cluster centers in red
  for (const auto & cluster : clusters) {
    float sum_x = 0, sum_y = 0;
    for (const auto & pt : cluster) {
      sum_x += pt.x;
      sum_y += pt.y;
    }
    cv::Point center(sum_x / cluster.size(), sum_y / cluster.size());
    cv::circle(debug_img, center, 8, cv::Scalar(0, 0, 255), 2);
  }
  
  cv::imwrite("/tmp/rack_detection_debug.png", debug_img);
  RCLCPP_INFO(this->get_logger(), "Debug image saved: /tmp/rack_detection_debug.png");
  
  return racks;
}

void RackDetectorNode::publishMarkers(
  const std::vector<RackPose> & racks,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray rack_markers;
  visualization_msgs::msg::MarkerArray waypoint_markers;
  
  for (size_t i = 0; i < racks.size(); ++i) {
    const auto & rack = racks[i];
    
    // Rack cube
    visualization_msgs::msg::Marker rack_marker;
    rack_marker.header = header;
    rack_marker.header.stamp = this->now();
    rack_marker.ns = "racks";
    rack_marker.id = i;
    rack_marker.type = visualization_msgs::msg::Marker::CUBE;
    rack_marker.action = visualization_msgs::msg::Marker::ADD;
    rack_marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    rack_marker.pose.position.x = rack.x;
    rack_marker.pose.position.y = rack.y;
    rack_marker.pose.position.z = rack_height_ / 2.0;
    
    rack_marker.pose.orientation.z = std::sin(rack.theta / 2.0);
    rack_marker.pose.orientation.w = std::cos(rack.theta / 2.0);
    
    rack_marker.scale.x = rack.width;
    rack_marker.scale.y = rack.depth;
    rack_marker.scale.z = rack_height_;
    
    rack_marker.color.r = 0.0;
    rack_marker.color.g = 1.0;
    rack_marker.color.b = 0.0;
    rack_marker.color.a = 0.5;
    
    rack_markers.markers.push_back(rack_marker);
    
    // Waypoint
    visualization_msgs::msg::Marker waypoint_marker;
    waypoint_marker.header = rack_marker.header;
    waypoint_marker.ns = "waypoints";
    waypoint_marker.id = i;
    waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
    waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
    waypoint_marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    double offset = waypoint_offset_;
    waypoint_marker.pose.position.x = rack.x + offset * std::cos(rack.theta);
    waypoint_marker.pose.position.y = rack.y + offset * std::sin(rack.theta);
    waypoint_marker.pose.position.z = 0.0;
    
    waypoint_marker.pose.orientation = rack_marker.pose.orientation;
    
    waypoint_marker.scale.x = 0.3;
    waypoint_marker.scale.y = 0.05;
    waypoint_marker.scale.z = 0.05;
    
    waypoint_marker.color.r = 1.0;
    waypoint_marker.color.g = 0.0;
    waypoint_marker.color.b = 0.0;
    waypoint_marker.color.a = 0.8;
    
    waypoint_markers.markers.push_back(waypoint_marker);
    
    // Text label
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = rack_marker.header;
    text_marker.ns = "labels";
    text_marker.id = i;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    text_marker.pose.position.x = rack.x;
    text_marker.pose.position.y = rack.y;
    text_marker.pose.position.z = rack_height_ + 0.2;
    
    text_marker.text = "RACK " + std::to_string(i + 1);
    text_marker.scale.z = 0.15;
    
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    rack_markers.markers.push_back(text_marker);
  }
  
  rack_marker_pub_->publish(rack_markers);
  waypoint_marker_pub_->publish(waypoint_markers);
}

}  // namespace warehouse_rover_rack_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_rover_rack_detection::RackDetectorNode>(
    rclcpp::NodeOptions()
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
