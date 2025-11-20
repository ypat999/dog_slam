cmake_minimum_required(VERSION 3.5)
project(nav2_dog_slam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

cmake_policy(SET CMP0074 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Python script for initial pose publishing (install as script, not as executable target)




# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(
  DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}/
  PATTERN "*.py"
)

# Install web interface files
install(
  DIRECTORY web
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python script as executable
install(
  PROGRAMS scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
