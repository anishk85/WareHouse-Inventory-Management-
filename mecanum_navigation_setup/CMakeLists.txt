cmake_minimum_required(VERSION 3.8)
project(mecanum_navigation_setup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(PROGRAMS
  scripts/create_keepout_mask.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()