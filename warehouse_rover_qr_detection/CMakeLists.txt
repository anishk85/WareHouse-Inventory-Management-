cmake_minimum_required(VERSION 3.10)
project(warehouse_rover_qr_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(warehouse_rover_msgs REQUIRED)

find_library(ZBAR_LIBRARIES NAMES zbar)
find_path(ZBAR_INCLUDE_DIRS NAMES zbar.h)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

# QR Parser Library
add_library(qr_parser_lib src/qr_parser.cpp)
target_include_directories(qr_parser_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# QR Detector Node
add_executable(qr_detector_node src/qr_detector_node.cpp)

ament_target_dependencies(qr_detector_node
  rclcpp sensor_msgs cv_bridge warehouse_rover_msgs
)

target_link_libraries(qr_detector_node
  qr_parser_lib
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)

install(TARGETS qr_parser_lib qr_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(qr_parser_lib)

ament_package()
