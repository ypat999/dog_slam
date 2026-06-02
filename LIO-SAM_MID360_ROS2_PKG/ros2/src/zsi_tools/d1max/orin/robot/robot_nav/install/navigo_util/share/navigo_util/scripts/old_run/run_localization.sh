#! /bin/bash

source /opt/ros/humble/setup.bash
source /home/jszr/jszr_workspace/install/setup.bash
export ROS_DOMAIN_ID=24
export ROS_DISCOVERY_SERVER=192.168.133.1:20000

bash /home/jszr/jszr_workspace/install/localization/run_localization.sh
