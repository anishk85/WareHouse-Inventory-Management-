#include "mecanum_hardware/mecanum_stepper_interface.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>

// Serial communication
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecanum_hardware
{

hardware_interface::CallbackReturn MecanumStepperInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get stepper parameters
  steps_per_revolution_ = std::stod(info_.hardware_parameters["steps_per_revolution"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  gear_ratio_ = std::stod(info_.hardware_parameters.count("gear_ratio") ? 
                          info_.hardware_parameters["gear_ratio"] : "1.0");
  
  // Get IMU usage
  use_imu_ = info_.hardware_parameters.count("use_imu") && 
             info_.hardware_parameters["use_imu"] == "true";
  
  RCLCPP_INFO(
    rclcpp::get_logger("MecanumStepperInterface"),
    "Stepper params: steps/rev=%.0f, wheel_r=%.3f, gear_ratio=%.2f",
    steps_per_revolution_, wheel_radius_, gear_ratio_);

  // Setup ESP connections (2 ESPs)
  // ESP1: FL (index 0), FR (index 1)
  // ESP2: BL (index 2), BR (index 3)
  
  ESPConnection esp1;
  esp1.port = info_.hardware_parameters["esp1_port"];
  esp1.baudrate = std::stoi(info_.hardware_parameters["esp_baudrate"]);
  esp1.motor_indices = {0, 1};  // Front left, Front right
  esp_connections_.push_back(esp1);
  
  ESPConnection esp2;
  esp2.port = info_.hardware_parameters["esp2_port"];
  esp2.baudrate = esp1.baudrate;
  esp2.motor_indices = {2, 3};  // Back left, Back right
  esp_connections_.push_back(esp2);
  
  RCLCPP_INFO(
    rclcpp::get_logger("MecanumStepperInterface"),
    "ESP1: %s (FL, FR) | ESP2: %s (BL, BR)",
    esp1.port.c_str(), esp2.port.c_str());

  // Initialize wheel data structures (4 wheels)
  wheel_names_.resize(info_.joints.size());
  hw_commands_velocity_.resize(info_.joints.size(), 0.0);
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); i++) {
    wheel_names_[i] = info_.joints[i].name;
    RCLCPP_INFO(
      rclcpp::get_logger("MecanumStepperInterface"),
      "Joint %zu: %s", i, wheel_names_[i].c_str());
  }
  
  first_read_ = true;
  imu_angular_velocity_z_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumStepperInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), "Configuring...");
  
  // Open both ESP connections
  for (auto& esp : esp_connections_) {
    if (!open_esp_port(esp)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("MecanumStepperInterface"),
        "Failed to open ESP port: %s", esp.port.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("MecanumStepperInterface"),
      "Opened ESP port: %s @ %d baud", esp.port.c_str(), esp.baudrate);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), 
              "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MecanumStepperInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        wheel_names_[i], 
        hardware_interface::HW_IF_POSITION, 
        &hw_states_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        wheel_names_[i], 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_states_velocity_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MecanumStepperInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        wheel_names_[i], 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_commands_velocity_[i]));
  }
  
  return command_interfaces;
}

hardware_interface::CallbackReturn MecanumStepperInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), "Activating...");
  
  // Send zero velocity to all motors
  std::vector<double> zero_cmd = {0.0, 0.0};
  for (auto& esp : esp_connections_) {
    send_velocity_command(esp, zero_cmd);
  }
  
  first_read_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), 
              "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumStepperInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), "Deactivating...");
  
  // Stop all motors
  std::vector<double> zero_cmd = {0.0, 0.0};
  for (auto& esp : esp_connections_) {
    send_velocity_command(esp, zero_cmd);
  }
  
  // Close ESP ports
  for (auto& esp : esp_connections_) {
    close_esp_port(esp);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), 
              "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumStepperInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (first_read_) {
    last_read_time_ = time;
    first_read_ = false;
    return hardware_interface::return_type::OK;
  }
  
  // Calculate odometry based on commanded velocities
  // Since steppers are accurate, we assume commanded = actual
  calculate_odometry(period);
  
  last_read_time_ = time;
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumStepperInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send velocity commands to ESPs
  // ESP1 controls motors 0,1 (FL, FR)
  // ESP2 controls motors 2,3 (BL, BR)
  
  for (auto& esp : esp_connections_) {
    std::vector<double> motor_cmds;
    for (int idx : esp.motor_indices) {
      motor_cmds.push_back(hw_commands_velocity_[idx]);
    }
    send_velocity_command(esp, motor_cmds);
  }
  
  return hardware_interface::return_type::OK;
}

// ========== ESP SERIAL COMMUNICATION ==========

bool MecanumStepperInterface::open_esp_port(ESPConnection& esp)
{
  esp.fd = ::open(esp.port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  
  if (esp.fd == -1) {
    return false;
  }
  
  struct termios options;
  tcgetattr(esp.fd, &options);
  
  // Set baud rate
  speed_t baud;
  switch (esp.baudrate) {
    case 9600:   baud = B9600; break;
    case 19200:  baud = B19200; break;
    case 38400:  baud = B38400; break;
    case 57600:  baud = B57600; break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:     baud = B115200; break;
  }
  
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);
  
  // 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  
  // No flow control
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag |= CREAD | CLOCAL;
  
  // Raw mode
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;
  
  // Set timeout
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 1;
  
  tcsetattr(esp.fd, TCSANOW, &options);
  tcflush(esp.fd, TCIOFLUSH);
  
  // Wait for ESP to be ready
  usleep(2000000);  // 2 seconds for ESP to boot
  
  return true;
}

void MecanumStepperInterface::close_esp_port(ESPConnection& esp)
{
  if (esp.fd != -1) {
    ::close(esp.fd);  // Use :: to specify global namespace
    esp.fd = -1;
  }
}

bool MecanumStepperInterface::send_velocity_command(
  ESPConnection& esp, 
  const std::vector<double>& velocities)
{
  // Protocol: "V,steps_per_sec_motor1,steps_per_sec_motor2\n"
  // Convert rad/s to steps/s for steppers
  
  std::ostringstream oss;
  oss << "V";
  
  for (double vel_rad_s : velocities) {
    double steps_per_sec = velocity_to_steps_per_sec(vel_rad_s);
    oss << "," << std::fixed << std::setprecision(1) << steps_per_sec;
  }
  oss << "\n";
  
  std::string message = oss.str();
  // Use :: to specify global namespace write() function
  ssize_t bytes_written = ::write(esp.fd, message.c_str(), message.length());
  
  if (bytes_written != static_cast<ssize_t>(message.length())) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("MecanumStepperInterface"),
      *rclcpp::Clock::make_shared(),
      1000,
      "Failed to write to ESP port %s", esp.port.c_str());
    return false;
  }
  
  return true;
}

void MecanumStepperInterface::calculate_odometry(const rclcpp::Duration & period)
{
  double dt = period.seconds();
  
  // Update positions based on commanded velocities (steppers are accurate)
  for (size_t i = 0; i < hw_commands_velocity_.size(); i++) {
    hw_states_velocity_[i] = hw_commands_velocity_[i];
    hw_states_position_[i] += hw_commands_velocity_[i] * dt;
  }
}

double MecanumStepperInterface::velocity_to_steps_per_sec(double rad_per_sec)
{
  // Convert rad/s to steps/s
  // rad/s -> rev/s -> steps/s
  double rev_per_sec = rad_per_sec / (2.0 * M_PI);
  double steps_per_sec = rev_per_sec * steps_per_revolution_ * gear_ratio_;
  return steps_per_sec;
}

}  // namespace mecanum_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanum_hardware::MecanumStepperInterface, 
  hardware_interface::SystemInterface)