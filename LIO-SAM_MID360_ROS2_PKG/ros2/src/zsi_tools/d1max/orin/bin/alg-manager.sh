#!/bin/bash

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# ensure /ota/alg_data/map exists for robot_slam
mkdir -p /ota/alg_data/map
mkdir -p /ota/alg_data/map/map.static

robot_alg_manager zg-arc
