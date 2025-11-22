// Copyright 2024 Mecanum Hardware
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MECANUM_HARDWARE__MECANUM_STEPPER_INTERFACE_HPP_
#define MECANUM_HARDWARE__MECANUM_STEPPER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mecanum_hardware
{

/// \brief ESP32 connection structure
struct ESPConnection
{
  std::string port;              // Serial port (e.g., /dev/ttyUSB0)
  int baudrate;                  // Baudrate (default: 115200)
  int fd;                        // File descriptor for serial port
  std::vector<int> motor_indices; // Motor indices controlled by this ESP (0=FL, 1=FR, 2=BL, 3=BR)
  
  ESPConnection() : baudrate(115200), fd(-1) {}
};

/// \brief Hardware interface for mecanum robot with NEMA23 stepper motors
///
/// This hardware interface communicates with two ESP32 microcontrollers
/// to control 4 NEMA23 stepper motors (no encoders) for a mecanum drive robot.
/// 
/// Hardware configuration:
/// - ESP1: Controls front-left and front-right motors via stepper drivers
/// - ESP2: Controls back-left and back-right motors via stepper drivers
/// - IMU: Connected to Raspberry Pi 5 via GPIO (optional, for sensor fusion)
/// - LiDAR: RPLidar A1 connected via USB to Raspberry Pi 5
/// - Depth Camera: RealSense D435/D455 (optional)
///
/// Communication protocol with ESP32:
/// - Command format: "V,steps_per_sec_motor1,steps_per_sec_motor2\n"
/// - Example: "V,100.0,-50.5\n" (motor1 forward @ 100 steps/s, motor2 backward @ 50.5 steps/s)
///
/// TF tree during operation:
/// - Mapping: odom -> base_link (published by cartographer using IMU + wheel odometry)
/// - Navigation: map -> odom (AMCL), odom -> base_link (cartographer with sensor fusion)
///
class MecanumStepperInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumStepperInterface)

  /// \brief Initialize the hardware interface
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// \brief Configure the hardware interface (open serial ports, setup IMU subscriber)
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /// \brief Cleanup the hardware interface
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /// \brief Export state interfaces (position, velocity for each wheel)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// \brief Export command interfaces (velocity for each wheel)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// \brief Activate the hardware (zero motors, prepare for operation)
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// \brief Deactivate the hardware (stop motors, close serial ports)
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// \brief Read sensor data and calculate odometry
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// \brief Write velocity commands to motors
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ========== ESP32 Serial Communication ==========
  
  /// \brief Open serial port to ESP32
  /// \param esp ESP connection structure
  /// \return true if successful, false otherwise
  bool open_esp_port(ESPConnection& esp);
  
  /// \brief Close serial port to ESP32
  /// \param esp ESP connection structure
  void close_esp_port(ESPConnection& esp);
  
  /// \brief Send velocity command to ESP32
  /// \param esp ESP connection structure
  /// \param velocities Vector of velocities (rad/s) for motors controlled by this ESP
  /// \return true if successful, false otherwise
  bool send_velocity_command(ESPConnection& esp, const std::vector<double>& velocities);
  
  /// \brief Read feedback from ESP32 (if available)
  /// \param esp ESP connection structure
  /// \param buffer Buffer to store received data
  /// \param max_bytes Maximum bytes to read
  /// \return Number of bytes read
  ssize_t read_esp_feedback(ESPConnection& esp, char* buffer, size_t max_bytes);
  
  // ========== Odometry Calculation ==========
  
  /// \brief Calculate odometry from wheel velocities and IMU data
  /// \param period Time period since last update
  void calculate_odometry(const rclcpp::Duration & period);
  
  /// \brief Convert angular velocity (rad/s) to steps per second
  /// \param rad_per_sec Angular velocity in rad/s
  /// \return Steps per second
  double velocity_to_steps_per_sec(double rad_per_sec);
  
  // ========== Hardware Parameters ==========
  
  // Stepper motor parameters (NEMA23)
  double steps_per_revolution_;  // Typically 200 for 1.8° step angle
  double wheel_radius_;          // Wheel radius in meters (e.g., 0.05m = 5cm)
  double gear_ratio_;            // Gear ratio (1.0 = direct drive)
  
  // Robot dimensions (loaded from URDF or config)
  double wheel_separation_x_;    // Front-back wheel separation (m)
  double wheel_separation_y_;    // Left-right wheel separation (m)
  
  // IMU fusion
  bool use_imu_;                 // Enable IMU fusion for better odometry
  double imu_angular_velocity_z_; // Latest IMU angular velocity (yaw rate)
  
  // ========== ESP32 Connections ==========
  
  std::vector<ESPConnection> esp_connections_;  // Two ESP32 controllers
  
  // ========== Wheel State ==========
  
  std::vector<std::string> wheel_names_;        // Wheel joint names
  std::vector<double> hw_commands_velocity_;    // Commanded velocities (rad/s)
  std::vector<double> hw_states_position_;      // Wheel positions (rad)
  std::vector<double> hw_states_velocity_;      // Wheel velocities (rad/s)
  
  // ========== ROS2 Node for IMU Subscription ==========
  
  rclcpp::Node::SharedPtr node_;                // ROS2 node for IMU subscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  
  // ========== Timing ==========
  
  rclcpp::Time last_read_time_;                 // Last time read() was called
  bool first_read_;                             // Flag for first read
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__MECANUM_STEPPER_INTERFACE_HPP_
