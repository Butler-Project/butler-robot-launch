cmake_minimum_required(VERSION 3.8)
project(robot_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Component library (for composition)
add_library(footprint_publisher_component SHARED src/footprint_publisher.cpp)
target_compile_definitions(footprint_publisher_component PRIVATE COMPONENT_ONLY)
ament_target_dependencies(footprint_publisher_component
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
rclcpp_components_register_nodes(footprint_publisher_component "FootprintPublisher")

# Standalone executable (for debugging / backward compat)
add_executable(footprint_publisher src/footprint_publisher.cpp)
ament_target_dependencies(footprint_publisher
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS footprint_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS footprint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# install(PROGRAMS
#   scripts/footprint_publisher.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
