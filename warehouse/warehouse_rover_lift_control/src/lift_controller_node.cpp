#include "warehouse_rover_lift_control/lift_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_rover_lift_control::LiftController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
