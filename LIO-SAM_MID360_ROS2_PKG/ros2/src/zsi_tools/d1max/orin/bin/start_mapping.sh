#! /bin/bash

. /opt/runtime/env.bash

source /opt/robot/robot_slam/install/setup.bash

export SLAM_TYPE=nx_zg
export USE_CUSTOM_MAP_PATH=false

/opt/robot/robot_slam/install/robot_slam/lib/robot_slam/robot_slam

