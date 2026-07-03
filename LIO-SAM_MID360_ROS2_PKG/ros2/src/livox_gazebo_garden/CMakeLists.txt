cmake_minimum_required(VERSION 3.8)
project(livox_gazebo_garden)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)


find_package(sdformat12 REQUIRED)
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(gz-sim7 REQUIRED)
find_package(gz-sensors7 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(gz-math7 REQUIRED)

include_directories(include)

set(GZ_INCLUDE_DIRS
  /usr/include/gz/sim7
  /usr/include/gz/sensors7
  /usr/include/gz/transport12
  /usr/include/gz/msgs9
  /usr/include/gz/math7
  /usr/include/gz/sdformat12
)

add_library(livox_garden_gpu_laser SHARED
  src/livox_garden_gpu_laser.cpp
)

target_include_directories(livox_garden_gpu_laser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GZ_INCLUDE_DIRS}
)

ament_target_dependencies(livox_garden_gpu_laser
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  ament_index_cpp
  livox_ros_driver2
)

target_link_libraries(livox_garden_gpu_laser
  gz-sim7::gz-sim7 gz-sensors7::gz-sensors7 gz-transport12::gz-transport12 gz-msgs9::gz-msgs9 gz-math7::gz-math7
)

add_library(livox_garden_cpu_lidar SHARED
  src/livox_garden_cpu_lidar.cpp
)

target_include_directories(livox_garden_cpu_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GZ_INCLUDE_DIRS}
)

ament_target_dependencies(livox_garden_cpu_lidar
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
  livox_ros_driver2
)

target_link_libraries(livox_garden_cpu_lidar
  gz-sim7::gz-sim7 gz-sensors7::gz-sensors7 gz-transport12::gz-transport12 gz-msgs9::gz-msgs9 gz-math7::gz-math7
)

install(TARGETS
  livox_garden_gpu_laser
  livox_garden_cpu_lidar
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config worlds urdf scan_mode rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  livox_garden_gpu_laser
  livox_garden_cpu_lidar
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  gz-sim7
  gz-sensors7
  gz-transport12
)

ament_package()
