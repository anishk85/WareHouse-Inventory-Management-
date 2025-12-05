#include "warehouse_rover_mission_control/mission_controller.hpp"
#include <chrono>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


using namespace std::chrono_literals;

namespace warehouse_rover_mission_control
{

MissionController::MissionController(const rclcpp::NodeOptions & options)
: Node("mission_controller", options),
  state_(MissionState::IDLE),
  current_rack_idx_(0),
  current_shelf_idx_(0),
  qr_detected_flag_(false)
{
  // Parameters
  this->declare_parameter<double>("scan_dwell_time", 3.0);
  this->declare_parameter<double>("lift_move_time", 2.0);
  
  this->get_parameter("scan_dwell_time", scan_dwell_time_);
  this->get_parameter("lift_move_time", lift_move_time_);
  
  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    "navigate_to_pose"
  );
  
  // Lift controller publisher
  lift_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/lift_position_controller/joint_trajectory",
    10
  );
  
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/mission_waypoints_markers", 10);

  marker_timer_ = this->create_wall_timer(
    1s,
    std::bind(&MissionController::publishWaypointMarkers, this)
  );


  
  // QR detection subscriber
  qr_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/qr_detections/status",
    10,
    std::bind(&MissionController::qrDetectionCallback, this, std::placeholders::_1)
  );
  
  // Load waypoints
  loadWaypoints();
  publishWaypointMarkers();
  
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
  RCLCPP_INFO(this->get_logger(), "  Mission Controller Started");
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
  RCLCPP_INFO(this->get_logger(), "Loaded %zu rack waypoints", waypoints_.size());
  RCLCPP_INFO(this->get_logger(), "Scan dwell time: %.1fs", scan_dwell_time_);
  RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
  
  // Wait for Nav2
  if (!nav_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
    RCLCPP_WARN(this->get_logger(), "Make sure Nav2 is running!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Nav2 connected! Ready to start mission.");
  }
}

void MissionController::loadWaypoints()
{
  // MEASURED RACK POSITIONS
  RackWaypoint r1;
  r1.name = "RACK_1";
  r1.x = -1.787;
  r1.y = 3.949;
  r1.theta = 0.0 * M_PI / 180.0;
  r1.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r1);
  
  RackWaypoint r2;
  r2.name = "RACK_2";
  r2.x = -0.582;
  r2.y = 3.960;
  r2.theta = 0.0 * M_PI / 180.0;
  r2.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r2);
  
  RackWaypoint r3;
  r3.name = "RACK_3";
  r3.x = -0.322;
  r3.y = 3.634;
  r3.theta = 270.0 * M_PI / 180.0;
  r3.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r3);
  
  RackWaypoint r4;
  r4.name = "RACK_4";
  r4.x = -0.308;
  r4.y = 2.575;
  r4.theta = 270.0 * M_PI / 180.0;
  r4.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r4);
  
  RackWaypoint r5;
  r5.name = "RACK_5";
  r5.x = -0.205;
  r5.y = 1.463;
  r5.theta = 270.0 * M_PI / 180.0;
  r5.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r5);
  
  RCLCPP_INFO(this->get_logger(), "Loaded %zu racks from map measurements", waypoints_.size());
}

void MissionController::startMission()
{
  if (waypoints_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No waypoints loaded!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
  RCLCPP_INFO(this->get_logger(), "  STARTING WAREHOUSE SCAN MISSION");
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
  RCLCPP_INFO(this->get_logger(), "Total racks: %zu", waypoints_.size());
  RCLCPP_INFO(this->get_logger(), "Total shelves: %zu per rack", 
              waypoints_[0].shelf_heights.size());
  
  current_rack_idx_ = 0;
  current_shelf_idx_ = 0;
  state_ = MissionState::IDLE;
  
  executeNextRack();
}

void MissionController::executeNextRack()
{
  if (current_rack_idx_ >= waypoints_.size()) {
    state_ = MissionState::MISSION_COMPLETE;
    RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
    RCLCPP_INFO(this->get_logger(), "  MISSION COMPLETE!");
    RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
    RCLCPP_INFO(this->get_logger(), "Scanned %zu racks successfully", waypoints_.size());
    return;
  }
  
  const auto & rack = waypoints_[current_rack_idx_];
  
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
  RCLCPP_INFO(this->get_logger(), "  RACK %zu/%zu: %s", 
              current_rack_idx_ + 1, waypoints_.size(), rack.name.c_str());
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
  
  current_shelf_idx_ = 0;
  navigateToRack(rack);
}

void MissionController::navigateToRack(const RackWaypoint & rack)
{
  double d = 0.4; 
  state_ = MissionState::NAVIGATING_TO_RACK;
  
  RCLCPP_INFO(this->get_logger(), "Navigating to: (%.2f, %.2f) theta=%.0f deg",
              rack.x, rack.y, rack.theta * 180.0 / M_PI);
  
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  
  goal_msg.pose.pose.position.x = rack.x;
  goal_msg.pose.pose.position.y = rack.y;
  goal_msg.pose.pose.position.z = 0.0;
  
  goal_msg.pose.pose.orientation.z = std::sin(rack.theta / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(rack.theta / 2.0);
  
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&MissionController::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&MissionController::feedbackCallback, this, 
              std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&MissionController::resultCallback, this, std::placeholders::_1);
  
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionController::publishWaypointMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto &rack : waypoints_)
    {
        // -------------------------
        // Sphere for waypoint position
        // -------------------------
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = now();
        sphere.ns = "waypoints";
        sphere.id = id++;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;

        sphere.pose.position.x = rack.x;
        sphere.pose.position.y = rack.y;
        sphere.pose.position.z = 0.1;

        sphere.scale.x = 0.2;
        sphere.scale.y = 0.2;
        sphere.scale.z = 0.2;

        sphere.color.r = 1.0;
        sphere.color.g = 1.0;
        sphere.color.b = 0.0;
        sphere.color.a = 1.0;

        sphere.lifetime = rclcpp::Duration(0,0);
        marker_array.markers.push_back(sphere);

        // -------------------------
        // Arrow for orientation
        // -------------------------
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = now();
        arrow.ns = "waypoints";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.pose.position.x = rack.x;
        arrow.pose.position.y = rack.y;
        arrow.pose.position.z = 0.1;

        // MUST SET full quaternion
        double half = rack.theta / 2.0;
        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = std::sin(half);
        arrow.pose.orientation.w = std::cos(half);

        arrow.scale.x = 0.5;  // length
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;

        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 1.0;
        arrow.color.a = 1.0;

        arrow.lifetime = rclcpp::Duration(0,0);
        marker_array.markers.push_back(arrow);

        // -------------------------
        // Text label
        // -------------------------
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = now();
        text.ns = "waypoints_text";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        text.pose.position.x = rack.x;
        text.pose.position.y = rack.y;
        text.pose.position.z = 0.5;

        text.scale.z = 0.25;

        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;

        text.text = rack.name;

        text.lifetime = rclcpp::Duration(0,0);
        marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);
}



void MissionController::moveLiftToHeight(double height)
{
  state_ = MissionState::ADJUSTING_LIFT;
  
  RCLCPP_INFO(this->get_logger(), "  Moving lift to height: %.2fm", height);
  
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"lift_joint"};
  
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {height};
  point.time_from_start = rclcpp::Duration::from_seconds(lift_move_time_);
  double wait_time = 60.0;
  
  traj.points.push_back(point);
  
  lift_pub_->publish(traj);
  
  // Create timer to wait for lift movement + scanning
  auto total_time = lift_move_time_ + scan_dwell_time_;
  mission_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(wait_time),
    [this]() {
      mission_timer_->cancel();  // One-shot timer
      this->processNextShelf();
    }
  );
  
  // Start scanning state immediately
  state_ = MissionState::SCANNING_QR;
  const auto & rack = waypoints_[current_rack_idx_];
  RCLCPP_INFO(this->get_logger(), "  Scanning shelf %zu/%zu at height %.2fm...",
              current_shelf_idx_ + 1, rack.shelf_heights.size(), height);
  
  qr_detected_flag_ = false;
}

void MissionController::waitForQRDetection()
{
  // This function is no longer used - scanning happens via timer
}

void MissionController::processNextShelf()
{
  const auto & rack = waypoints_[current_rack_idx_];
  
  if (qr_detected_flag_) {
    RCLCPP_INFO(this->get_logger(), "  ✓ QR code detected!");
  } else {
    RCLCPP_WARN(this->get_logger(), "  ⚠ No QR code detected");
  }
  
  current_shelf_idx_++;
  
  if (current_shelf_idx_ >= rack.shelf_heights.size()) {
    state_ = MissionState::RACK_COMPLETE;
    RCLCPP_INFO(this->get_logger(), "✓ Rack %s complete!", rack.name.c_str());
    
    current_rack_idx_++;
    executeNextRack();
  } else {
    double next_height = rack.shelf_heights[current_shelf_idx_];
    moveLiftToHeight(next_height);
  }
}

void MissionController::goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Navigation goal rejected!");
    state_ = MissionState::FAILED;
  } else {
    RCLCPP_INFO(this->get_logger(), "  Navigation goal accepted");
  }
}

void MissionController::feedbackCallback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  auto current_pose = feedback->current_pose.pose;
  RCLCPP_DEBUG(this->get_logger(), 
    "  Position: (%.2f, %.2f)", 
    current_pose.position.x, current_pose.position.y);
}

void MissionController::resultCallback(const GoalHandleNav::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Arrived at rack!");
      
      if (!waypoints_.empty()) {
        double first_height = waypoints_[current_rack_idx_].shelf_heights[0];
        moveLiftToHeight(first_height);
      }
      break;
      
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Navigation aborted!");
      state_ = MissionState::FAILED;
      break;
      
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Navigation canceled");
      state_ = MissionState::FAILED;
      break;
      
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown navigation result");
      state_ = MissionState::FAILED;
      break;
  }
}

void MissionController::qrDetectionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (state_ == MissionState::SCANNING_QR) {
    if (msg->data == "detected") {
      qr_detected_flag_ = true;
    }
  }
}

}  // namespace warehouse_rover_mission_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<warehouse_rover_mission_control::MissionController>(
    rclcpp::NodeOptions()
  );
  
  node->startMission();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
