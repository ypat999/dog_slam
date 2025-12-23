cmake_minimum_required(VERSION 3.5)
project(sc_pgo_ros2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)   # usually needed alongside


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(alaserPGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
ament_target_dependencies(alaserPGO 
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros cv_bridge pcl_conversions pcl_msgs
)
target_compile_options(alaserPGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(alaserPGO 
  ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

install(TARGETS
  alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

install( DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch      
		 FILES_MATCHING PATTERN "*.launch.py" 
)

install(
  DIRECTORY rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
