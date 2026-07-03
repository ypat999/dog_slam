#!/bin/bash

if [ $# -lt 1 ]; then
  echo "Usage: "
  echo "./echo.sh [topic]"
  exit 1
fi

echo "./echo.sh $1"

JSZR_BASE_DIR=/home/jszr/jszr_workspace
source /opt/ros/humble/setup.bash
source ${JSZR_BASE_DIR}/install/setup.bash
export ROS_DOMAIN_ID=24
export ROS_DISCOVERY_SERVER=192.168.133.1:20000
export FASTRTPS_DEFAULT_PROFILES_FILE=${JSZR_BASE_DIR}/install/navigo_util/share/navigo_util/scripts/super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start

topic=$1

ros2 topic echo $topic

exit 0
