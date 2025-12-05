#ifndef WAREHOUSE_ROVER_LIFT_CONTROL__LIFT_CONTROLLER_HPP_
#define WAREHOUSE_ROVER_LIFT_CONTROL__LIFT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <chrono>

namespace warehouse_rover_lift_control
{

class LiftController : public rclcpp::Node
{
public:
  LiftController();

private:
  // Service handlers
  void handleScanCycleRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void handleResetRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  // Lift movement
  bool moveToHeight(double target_height);
  
  // ROS2 interfaces
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_cycle_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  
  // Parameters
  std::vector<double> shelf_heights_;
  double move_duration_;
  double dwell_time_;
  std::string joint_name_;
  std::string trajectory_topic_;
};

}  // namespace warehouse_rover_lift_control

#endif  // WAREHOUSE_ROVER_LIFT_CONTROL__LIFT_CONTROLLER_HPP_
