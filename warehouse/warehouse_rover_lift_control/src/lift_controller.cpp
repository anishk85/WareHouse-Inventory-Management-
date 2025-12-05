#include "warehouse_rover_lift_control/lift_controller.hpp"
#include <thread>

namespace warehouse_rover_lift_control
{

LiftController::LiftController()
: Node("lift_controller")
{
  // Declare parameters
  this->declare_parameter("joint_name", "lift_joint");
  this->declare_parameter("trajectory_topic", "/lift_position_controller/joint_trajectory");
  this->declare_parameter("shelf_heights", std::vector<double>{0.0, 0.25, 0.50, 0.75});
  this->declare_parameter("move_duration", 2.0);
  this->declare_parameter("dwell_time", 3.0);
  
  // Get parameters
  joint_name_ = this->get_parameter("joint_name").as_string();
  trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();
  shelf_heights_ = this->get_parameter("shelf_heights").as_double_array();
  move_duration_ = this->get_parameter("move_duration").as_double();
  dwell_time_ = this->get_parameter("dwell_time").as_double();
  
  // Create publisher
  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    trajectory_topic_, 10);
  
  // Create services
  scan_cycle_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/scan_cycle",
    std::bind(&LiftController::handleScanCycleRequest, this,
              std::placeholders::_1, std::placeholders::_2));
  
  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(&LiftController::handleResetRequest, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // Startup banner
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(), "║         WAREHOUSE LIFT CONTROLLER STARTED             ║");
  RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Joint name: %s", joint_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Topic: %s", trajectory_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Shelf heights: [%.2f, %.2f, %.2f, %.2f]",
              shelf_heights_[0], shelf_heights_[1], shelf_heights_[2], shelf_heights_[3]);
  RCLCPP_INFO(this->get_logger(), "  Move duration: %.1fs", move_duration_);
  RCLCPP_INFO(this->get_logger(), "  Dwell time: %.1fs", dwell_time_);
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "✅ Ready to scan shelves!");
  RCLCPP_INFO(this->get_logger(), " ");
  
  // Wait for publisher to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void LiftController::handleScanCycleRequest(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(), "║           STARTING FULL SHELF SCAN CYCLE              ║");
  RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════╝");
  
  // Scan each shelf
  for (size_t i = 0; i < shelf_heights_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "┌────────────────────────────────────────────────────────┐");
    RCLCPP_INFO(this->get_logger(), "│ 📦 SHELF %zu/%zu: HEIGHT %.2fm                         │",
                i + 1, shelf_heights_.size(), shelf_heights_[i]);
    RCLCPP_INFO(this->get_logger(), "└────────────────────────────────────────────────────────┘");
    
    // Move to shelf height
    RCLCPP_INFO(this->get_logger(), "⬆️  Moving lift to %.2fm...", shelf_heights_[i]);
    if (!moveToHeight(shelf_heights_[i])) {
      response->success = false;
      response->message = "Failed to move to shelf " + std::to_string(i + 1);
      return;
    }
    
    // Wait for movement to complete + dwell time for scanning
    double total_wait = move_duration_ + dwell_time_;
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting %.1fs (move + scan)...", total_wait);
    std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(total_wait * 1000)));
    
    RCLCPP_INFO(this->get_logger(), "✅ Shelf %zu complete", i + 1);
  }
  
  // Return to home position
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "🏠 Returning to home position (0.00m)...");
  if (!moveToHeight(0.0)) {
    response->success = false;
    response->message = "Failed to return to home position";
    return;
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(
    static_cast<int>(move_duration_ * 1000)));
  
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(), "║            ✅ SCAN CYCLE COMPLETE!                     ║");
  RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(this->get_logger(), " ");
  
  response->success = true;
  response->message = "Scanned " + std::to_string(shelf_heights_.size()) + " shelves successfully";
}

void LiftController::handleResetRequest(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "🔄 Reset requested - returning to home...");
  
  if (moveToHeight(0.0)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(move_duration_ * 1000)));
    response->success = true;
    response->message = "Lift reset to home position";
    RCLCPP_INFO(this->get_logger(), "✅ Reset complete");
  } else {
    response->success = false;
    response->message = "Failed to reset lift";
    RCLCPP_ERROR(this->get_logger(), "❌ Reset failed");
  }
}

bool LiftController::moveToHeight(double target_height)
{
  // Create trajectory message
  auto trajectory = trajectory_msgs::msg::JointTrajectory();
  trajectory.joint_names = {joint_name_};
  
  // Create trajectory point
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  point.positions = {target_height};
  point.velocities = {0.0};
  point.time_from_start.sec = static_cast<int>(move_duration_);
  point.time_from_start.nanosec = 0;
  
  trajectory.points = {point};
  
  // Publish trajectory
  trajectory_pub_->publish(trajectory);
  
  return true;
}

}  // namespace warehouse_rover_lift_control
