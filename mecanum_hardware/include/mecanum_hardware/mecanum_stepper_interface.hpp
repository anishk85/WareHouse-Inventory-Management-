#ifndef MECANUM_HARDWARE__MECANUM_STEPPER_INTERFACE_HPP_
#define MECANUM_HARDWARE__MECANUM_STEPPER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mecanum_hardware
{

class MecanumStepperInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumStepperInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ESP32 connection structure
  struct ESPConnection {
    std::string port;
    int baudrate;
    int fd = -1;  // File descriptor
    std::vector<int> motor_indices;  // Which motors this ESP controls
  };

  // ESP serial communication
  bool open_esp_port(ESPConnection& esp);
  void close_esp_port(ESPConnection& esp);
  bool send_velocity_command(ESPConnection& esp, const std::vector<double>& velocities);

  // Odometry calculation
  void calculate_odometry(const rclcpp::Duration & period);
  double velocity_to_steps_per_sec(double rad_per_sec);

  // Hardware parameters
  double steps_per_revolution_;
  double wheel_radius_;
  double gear_ratio_;
  bool use_imu_;

  // ESP connections (2 ESPs)
  std::vector<ESPConnection> esp_connections_;

  // Wheel state
  std::vector<std::string> wheel_names_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Timing
  rclcpp::Time last_read_time_;
  bool first_read_;

  // IMU data
  double imu_angular_velocity_z_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__MECANUM_STEPPER_INTERFACE_HPP_