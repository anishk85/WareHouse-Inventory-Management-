cmake_minimum_required(VERSION 3.16)
project(mecanum_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  mecanum_hardware
  SHARED
  src/mecanum_stepper_interface.cpp
)
target_compile_features(mecanum_hardware PUBLIC cxx_std_17)
target_include_directories(mecanum_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mecanum_hardware>
)
ament_target_dependencies(
  mecanum_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware.xml)

# INSTALL

# 1. Install Headers
install(
  DIRECTORY include/
  DESTINATION include/mecanum_hardware
)

# 2. Install Directories 
install(
  DIRECTORY config launch scripts firmware
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install the Plugin XML 
install(
  FILES mecanum_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install Python Scripts 
install(PROGRAMS
  scripts/imu_publisher_node.py
  scripts/magnetometer_publisher_node.py
  scripts/quick_serial_check.py
  scripts/joystick_teleop.py
  scripts/actuator_control_node.py
  scripts/actuator_pid_control_node.py
  scripts/laser_sensor.py
  scripts/waypoint_follower_node.py
  scripts/nav2_waypoint_follower.py
  scripts/teleop_with_waypoint_save.py
  scripts/ros2_web_bridge.py
  scripts/inventory_mission_node.py
  scripts/height_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# 5. Install the Library Target
install(TARGETS mecanum_hardware
  EXPORT export_mecanum_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_mecanum_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
