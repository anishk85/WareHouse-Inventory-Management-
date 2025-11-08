cmake_minimum_required(VERSION 3.8)
project(mecanum_in_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY arena
  DESTINATION share/${PROJECT_NAME}/
)
# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install mesh files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/mecanum_teleop_key.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()