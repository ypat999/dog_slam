#! /bin/bash

MAP="${1:-none}"
LOG_PATH=/home/jszr/.jszr/log

mkdir -p $LOG_PATH

source /opt/ros/humble/setup.bash
source /home/jszr/jszr_workspace/install/setup.bash
export ROS_DOMAIN_ID=24
export ROS_DISCOVERY_SERVER=192.168.133.1:20000

ros2 launch robot_nav2 navigation_bringup.launch.py platform:=NX_XG3588 map:="$MAP" mc_controller_type:=RL_TRACK_VELOCITY communication_type:=UDP 2>&1 | tee $LOG_PATH/navigation_"$(date +'%Y-%m-%d_%H-%M-%S')".log