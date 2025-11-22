#ifndef WAREHOUSE_ROVER_MISSION_CONTROL__MISSION_CONTROLLER_HPP_
#define WAREHOUSE_ROVER_MISSION_CONTROL__MISSION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>
#include <string>

namespace warehouse_rover_mission_control
{

struct RackWaypoint
{
  std::string name;
  double x;
  double y;
  double theta;
  std::vector<double> shelf_heights;
};

enum class MissionState
{
  IDLE,
  NAVIGATING_TO_RACK,
  ADJUSTING_LIFT,
  SCANNING_QR,
  MOVING_TO_NEXT_SHELF,
  RACK_COMPLETE,
  MISSION_COMPLETE,
  FAILED
};

class MissionController : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  
  explicit MissionController(const rclcpp::NodeOptions & options);
  ~MissionController() = default;
  
  void startMission();

private:
  void loadWaypoints();
  void executeNextRack();
  void navigateToRack(const RackWaypoint & rack);
  void moveLiftToHeight(double height);
  void waitForQRDetection();
  void processNextShelf();
  
  // Nav2 callbacks
  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNav::WrappedResult & result);
  
  // QR detection callback
  void qrDetectionCallback(const std_msgs::msg::String::SharedPtr msg);
  
  // State machine
  MissionState state_;
  size_t current_rack_idx_;
  size_t current_shelf_idx_;
  std::vector<RackWaypoint> waypoints_;
  
  // ROS
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lift_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  
  // Parameters
  double scan_dwell_time_;
  double lift_move_time_;
  bool qr_detected_flag_;
};

}  // namespace warehouse_rover_mission_control

#endif  // WAREHOUSE_ROVER_MISSION_CONTROL__MISSION_CONTROLLER_HPP_
