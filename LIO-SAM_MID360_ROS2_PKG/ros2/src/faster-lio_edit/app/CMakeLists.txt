add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
        faster_lio_lib gflags
        )
ament_target_dependencies(run_mapping_online
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  tf2
  tf2_ros
  livox_ros_driver2
)

add_executable(run_mapping_offline run_mapping_offline.cc)
target_link_libraries(run_mapping_offline
        faster_lio_lib gflags
        )
ament_target_dependencies(run_mapping_offline
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  tf2
  tf2_ros
  livox_ros_driver2
)

# Install executables
install(TARGETS 
  run_mapping_online
  run_mapping_offline
  DESTINATION lib/${PROJECT_NAME}
)