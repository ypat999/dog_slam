#! /bin/bash

. /opt/runtime/env.bash

source /opt/robot/robot-forward/install/setup.bash
export ROBOT_LOG_DIR="/home/robot/.robot"
export ROBOT_PARAM_DIR="/home/robot/.robot"
cd /opt/robot/robot-forward/install/robot_forward/lib/robot_forward && ./robot_forward