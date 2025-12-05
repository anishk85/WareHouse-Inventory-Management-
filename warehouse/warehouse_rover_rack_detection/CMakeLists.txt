cmake_minimum_required(VERSION 3.8)
project(warehouse_rover_rack_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Rack Detector Node
add_executable(rack_detector_node src/rack_detector_node.cpp)

target_include_directories(rack_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(rack_detector_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

target_link_libraries(rack_detector_node
  ${OpenCV_LIBS}
)

# Install
install(TARGETS
  rack_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
