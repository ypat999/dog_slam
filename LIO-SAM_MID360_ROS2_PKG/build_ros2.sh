#!/bin/bash
# ROS2 Packages
if [ -x "$(command -v ./ros2/src/livox_ros_driver2/build.sh)" ]; then
    ./ros2/src/livox_ros_driver2/build.sh humble
else
    echo "[livox_ros_driver2/build.sh] not found"
    exit 1
fi

cd ./ros2
colcon build --symlink-install --packages-select lio_sam
colcon build --symlink-install --packages-select fast_lio
colcon build --symlink-install --packages-skip livox_gazebo_ros2_gpu_simulation
source install/setup.bash
