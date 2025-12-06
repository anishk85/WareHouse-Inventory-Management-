cmake_minimum_required(VERSION 3.8)
project(warehouse_rover_database)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(warehouse_rover_msgs REQUIRED)
find_package(SQLite3 REQUIRED)

include_directories(
  include
  ${SQLite3_INCLUDE_DIRS}
)

# Database Manager Library
add_library(database_manager_lib src/database_manager.cpp)
target_link_libraries(database_manager_lib ${SQLite3_LIBRARIES})

# Inventory Node
add_executable(inventory_node src/inventory_node.cpp)

ament_target_dependencies(inventory_node
  rclcpp warehouse_rover_msgs
)

target_link_libraries(inventory_node database_manager_lib)

# Install executables and libraries
install(TARGETS database_manager_lib inventory_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_export_include_directories(include)
ament_export_libraries(database_manager_lib)

ament_package()
