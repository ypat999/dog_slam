cmake_minimum_required(VERSION 3.5)
project(lio_sam_livox)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)

# Find GTSAM
find_package(GTSAM REQUIRED)
if(NOT GTSAM_FOUND)
  message(WARNING "GTSAM not found. LIO-SAM will be built without GTSAM support.")
  set(GTSAM_FOUND FALSE)
else()
  set(GTSAM_FOUND TRUE)
endif()

# ROS2 message generation
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message files
set(msg_files
  "msg/CloudInfo.msg"
)

set(srv_files
  "srv/SaveMap.srv"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs
  ADD_LINTER_TESTS
)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Create a variable for source files
set(LIO_SAM_SOURCES
  src/imageProjection.cpp
  src/featureExtraction.cpp
  src/imuPreintegration.cpp
  src/mapOptmization.cpp
)

# Build executables
foreach(source_file ${LIO_SAM_SOURCES})
  get_filename_component(exec_name ${source_file} NAME_WE)
  add_executable(${exec_name} ${source_file})
  
  # Set properties
  set_target_properties(${exec_name} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
  )
  
  # Include directories for this target
  target_include_directories(${exec_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
  
  # Link libraries
  ament_target_dependencies(${exec_name}
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    pcl_conversions
    pcl_ros
  )
  
  # Link message interfaces
  rosidl_target_interfaces(${exec_name} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  # Link PCL and other # Link libraries
  target_link_libraries(${exec_name}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
  )
  
  # Link GTSAM if found
  if(GTSAM_FOUND)
    target_link_libraries(${exec_name} gtsam)
    target_compile_definitions(${exec_name} PRIVATE GTSAM_FOUND)
  endif()
  
  # Install executable
  install(TARGETS ${exec_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY
  include/
  DESTINATION include/
)

# Generate ament index
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  pcl_ros
  PCL
  Eigen3
  Boost
  OpenCV
)

if(GTSAM_FOUND)
  ament_export_dependencies(GTSAM)
endif()

ament_package()