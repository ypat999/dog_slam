#! /bin/bash

. /opt/runtime/env.bash

export ZSIBOT_CACHE=/home/robot/.robot/cache
export ZSIBOT_CONFIG_ROOT=/opt/robot/perception/install/perception/config
export LD_LIBRARY_PATH=/opt/TensorRT/lib:/usr/lib/aarch64-linux-gnu/nvidia/:/opt/robot/perception/install/perception/lib:$LD_LIBRARY_PATH
export ROBOT_LOG_DIR="/home/robot/.robot"
export ROBOT_PARAM_DIR="/home/robot/.robot"

/opt/robot/perception/install/perception/bin/perception_jobs /opt/robot/perception/install/perception/config/perception/perception_job/perception_job.yaml
