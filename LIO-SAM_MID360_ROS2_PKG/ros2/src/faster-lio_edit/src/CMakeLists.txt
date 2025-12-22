add_library(faster_lio_lib
        laser_mapping.cc
        pointcloud_preprocess.cc
        options.cc
        utils.cc
        )

# Replace catkin dependencies
# add_dependencies(faster_lio_lib ${PROJECT_NAME}_gencpp livox_ros_driver_gencpp)

# Get the typesupport target for our custom message
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(faster_lio_lib
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        "${cpp_typesupport_target}"
        )

target_include_directories(faster_lio_lib PUBLIC
  ${PYTHON_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# Add ament dependencies
ament_target_dependencies(faster_lio_lib
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  tf2
  tf2_ros
  tf2_eigen
  livox_ros_driver2
)