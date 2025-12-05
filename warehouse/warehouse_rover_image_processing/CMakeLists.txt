cmake_minimum_required(VERSION 3.8)
project(warehouse_rover_image_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(warehouse_rover_msgs REQUIRED)
find_package(warehouse_rover_qr_detection REQUIRED)

# OpenCV with optional CUDA
find_package(OpenCV REQUIRED)

# Check if OpenCV has CUDA support
if(OpenCV_CUDA_VERSION)
  message(STATUS "OpenCV CUDA support found: ${OpenCV_CUDA_VERSION}")
  add_definitions(-DHAVE_OPENCV_CUDAARITHM)
  set(HAVE_CUDA TRUE)
else()
  message(STATUS "OpenCV CUDA support NOT found - using CPU fallback")
  set(HAVE_CUDA FALSE)
endif()

# ZBar
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

# Hardware Detector Library
add_library(hardware_detector_lib 
  src/hardware_detector.cpp
)
target_link_libraries(hardware_detector_lib
  ${OpenCV_LIBRARIES}
)

# Image Preprocessor Library
add_library(image_preprocessor_lib
  src/image_preprocessor.cpp
)
target_link_libraries(image_preprocessor_lib
  hardware_detector_lib
  ${OpenCV_LIBRARIES}
)
target_include_directories(image_preprocessor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Enhanced QR Detector Node
add_executable(qr_detector_enhanced_node
  src/qr_detector_enhanced_node.cpp
)

ament_target_dependencies(qr_detector_enhanced_node
  rclcpp
  sensor_msgs
  cv_bridge
  warehouse_rover_msgs
  warehouse_rover_qr_detection
)

target_link_libraries(qr_detector_enhanced_node
  image_preprocessor_lib
  hardware_detector_lib
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)

# Hardware info tool
add_executable(hardware_info_tool
  src/hardware_info_tool.cpp
)

target_link_libraries(hardware_info_tool
  hardware_detector_lib
  image_preprocessor_lib
  ${OpenCV_LIBRARIES}
)

# Install executables and libraries
install(TARGETS
  hardware_detector_lib
  image_preprocessor_lib
  qr_detector_enhanced_node
  hardware_info_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_libraries(hardware_detector_lib image_preprocessor_lib)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  warehouse_rover_msgs
  warehouse_rover_qr_detection
  OpenCV
)

ament_package()
