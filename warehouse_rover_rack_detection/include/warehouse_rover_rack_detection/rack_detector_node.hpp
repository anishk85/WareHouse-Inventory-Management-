#ifndef WAREHOUSE_ROVER_RACK_DETECTION__RACK_DETECTOR_NODE_HPP_
#define WAREHOUSE_ROVER_RACK_DETECTION__RACK_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace warehouse_rover_rack_detection
{

struct RackPose
{
  double x;
  double y;
  double theta;
  double width;
  double depth;
};

class RackDetectorNode : public rclcpp::Node
{
public:
  explicit RackDetectorNode(const rclcpp::NodeOptions & options);
  ~RackDetectorNode() = default;

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  std::vector<RackPose> detectRacks(const nav_msgs::msg::OccupancyGrid & map_msg);
  void publishMarkers(
    const std::vector<RackPose> & racks,
    const std_msgs::msg::Header & header);
  
  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rack_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_;
  
  // Parameters
  int threshold_;
  int min_area_;
  int max_area_;
  double min_aspect_ratio_;
  double max_aspect_ratio_;
  double waypoint_offset_;
  
  // Rack dimensions (real measurements)
  double rack_length_;
  double rack_width_;
  double rack_height_;
  std::vector<double> shelf_heights_;
  
  // State
  std::vector<RackPose> detected_racks_;
  bool first_detection_;
};

}  // namespace warehouse_rover_rack_detection

#endif  // WAREHOUSE_ROVER_RACK_DETECTION__RACK_DETECTOR_NODE_HPP_
