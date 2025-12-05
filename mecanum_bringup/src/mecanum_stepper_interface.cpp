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

  //stepper parameters
  steps_per_revolution_ = std::stod(info_.hardware_parameters["steps_per_revolution"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  gear_ratio_ = std::stod(info_.hardware_parameters.count("gear_ratio") ? 
                          info_.hardware_parameters["gear_ratio"] : "1.0");
  
  // Get robot dimensions (for mecanum kinematics)
  wheel_separation_x_ = std::stod(info_.hardware_parameters.count("wheel_separation_x") ?
                                  info_.hardware_parameters["wheel_separation_x"] : "0.25");
  wheel_separation_y_ = std::stod(info_.hardware_parameters.count("wheel_separation_y") ?
                                  info_.hardware_parameters["wheel_separation_y"] : "0.30");
  
  // Get IMU usage
  use_imu_ = info_.hardware_parameters.count("use_imu") && 
             info_.hardware_parameters["use_imu"] == "true";
  
  RCLCPP_INFO(
    rclcpp::get_logger("MecanumStepperInterface"),
    "Stepper params: steps/rev=%.0f, wheel_r=%.3f, gear_ratio=%.2f, use_imu=%s",
    steps_per_revolution_, wheel_radius_, gear_ratio_, use_imu_ ? "true" : "false");

  // Setup ESP connections (2 ESPs)
  // ESP1: FL (index 0), FR (index 1)
  // ESP2: BL (index 2), BR (index 3)
  
  // Setup ESP32 connections
  ESPConnection esp1;
  esp1.port = info_.hardware_parameters["esp1_port"];
  esp1.baudrate = info_.hardware_parameters.count("esp_baudrate") ? 
                  std::stoi(info_.hardware_parameters["esp_baudrate"]) : 115200;
  esp1.motor_indices = {0, 1};  // Front left, Front right
  esp_connections_.push_back(esp1);
  
  ESPConnection esp2;
  esp2.port = info_.hardware_parameters["esp2_port"];
  esp2.baudrate = esp1.baudrate;  // Use same baudrate as ESP1
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

  // Create IMU subscriber if enabled
  if (use_imu_) {
    RCLCPP_INFO(
      rclcpp::get_logger("MecanumStepperInterface"),
      "IMU fusion enabled - will subscribe to /imu/data");
  }

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
  
  // Create ROS node for IMU subscription if enabled
  if (use_imu_) {
    try {
      // Create node with specific QoS to match IMU publisher
      node_ = std::make_shared<rclcpp::Node>(
        "mecanum_hardware_imu_listener",
        rclcpp::NodeOptions().use_intra_process_comms(false)
      );
      
      // QoS profile matching the IMU publisher (RELIABLE, VOLATILE)
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);
      
      imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          // Store latest IMU angular velocity (yaw rate) for slip detection
          // Note: Cartographer handles full IMU fusion at a higher level
          // This is only used for detecting wheel slip in diagnostics
          imu_angular_velocity_z_ = msg->angular_velocity.z;
          
          // We trust Cartographer to do the full sensor fusion
          // Hardware interface only reports pure wheel odometry
        });
      
      RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), 
                  "IMU subscriber created on /imu/data with RELIABLE QoS");
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("MecanumStepperInterface"),
                  "Failed to create IMU subscriber: %s. Continuing without IMU.", e.what());
      use_imu_ = false;
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), 
              "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumStepperInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), "Cleaning up...");
  
  // Close ESP ports if they're still open
  for (auto& esp : esp_connections_) {
    if (esp.fd != -1) {
      close_esp_port(esp);
    }
  }
  
  // Reset node and subscriber
  if (imu_subscriber_) {
    imu_subscriber_.reset();
  }
  if (node_) {
    node_.reset();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("MecanumStepperInterface"), 
              "Successfully cleaned up!");
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
  
  // Spin the node to process IMU callbacks if enabled
  if (use_imu_ && node_) {
    rclcpp::spin_some(node_);
  }
  
  // Calculate odometry (with IMU fusion if available)
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
    ::close(esp.fd);
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
  
  // Use configured wheel separations
  const double wheel_separation_x = wheel_separation_x_;
  const double wheel_separation_y = wheel_separation_y_;
  
  // Get wheel velocities (rad/s)
  double v_fl = hw_commands_velocity_[0];  // Front Left
  double v_fr = hw_commands_velocity_[1];  // Front Right
  double v_bl = hw_commands_velocity_[2];  // Back Left
  double v_br = hw_commands_velocity_[3];  // Back Right
  
  // Forward kinematics: Convert wheel velocities to robot velocity for wheel odometry 
  // Although we will not use it in final odom calculation done by cartographer but can be later used for debugging 
  // Mecanum wheel kinematics:
  // vx = R/4 * (v_fl + v_fr + v_bl + v_br)
  // vy = R/4 * (-v_fl + v_fr + v_bl - v_br)
  // omega = R/(4*(lx+ly)) * (-v_fl + v_fr - v_bl + v_br)
  
  double vx_wheel = wheel_radius_ / 4.0 * (v_fl + v_fr + v_bl + v_br);
  double vy_wheel = wheel_radius_ / 4.0 * (-v_fl + v_fr + v_bl - v_br);
  double omega_wheel = wheel_radius_ / (4.0 * (wheel_separation_x + wheel_separation_y)) * 
                       (-v_fl + v_fr - v_bl + v_br);
  
  // Update joint states with PURE wheel odometry
  // Cartographer will fuse this with IMU at a higher level
  for (size_t i = 0; i < hw_commands_velocity_.size(); i++) {
    hw_states_velocity_[i] = hw_commands_velocity_[i];
    hw_states_position_[i] += hw_commands_velocity_[i] * dt;
  }

  // Optional: Still detect slip for diagnostics (but don't correct for it)
  if (use_imu_) {
    double omega_diff = std::abs(imu_angular_velocity_z_ - omega_wheel);
    if (omega_diff > 0.5) {  // 0.5 rad/s threshold
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("MecanumStepperInterface"),
        *rclcpp::Clock::make_shared(),
        5000,  // Warn every 5 seconds
        "⚠️ Potential wheel slip! IMU: %.2f rad/s, Wheels: %.2f rad/s (diff: %.2f)",
        imu_angular_velocity_z_, omega_wheel, omega_diff);
    }
  }


  static int counter = 0;
  if (++counter % 100 == 0) {  // Log every ~1 second
    RCLCPP_DEBUG(
      rclcpp::get_logger("MecanumStepperInterface"),
      "Wheel Odom: vx=%.3f, vy=%.3f, omega=%.3f rad/s",
      vx_wheel, vy_wheel, omega_wheel);
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

ssize_t MecanumStepperInterface::read_esp_feedback(
  ESPConnection& esp, 
  char* buffer, 
  size_t max_bytes)
{
  if (esp.fd == -1) {
    return -1;
  }
  
  ssize_t bytes_read = ::read(esp.fd, buffer, max_bytes);
  return bytes_read;
}

}  // namespace mecanum_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanum_hardware::MecanumStepperInterface, 
  hardware_interface::SystemInterface)