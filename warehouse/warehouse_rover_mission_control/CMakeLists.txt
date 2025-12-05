cmake_minimum_required(VERSION 3.8)
project(warehouse_rover_mission_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Mission Controller
add_executable(mission_controller_node src/mission_controller.cpp)

target_include_directories(mission_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mission_controller_node
  rclcpp
  rclcpp_action
  visualization_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  nav2_msgs
)

# Install
install(TARGETS
  mission_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# Install Python scripts
install(PROGRAMS
  scripts/measure_rack_positions.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/lift_service_node.py
  scripts/measure_rack_positions.py
  scripts/auto_initialize_amcl.py
  scripts/mission_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
