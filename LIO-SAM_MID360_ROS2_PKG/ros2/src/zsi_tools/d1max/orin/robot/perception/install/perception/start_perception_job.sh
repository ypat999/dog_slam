#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd $SCRIPT_DIR

MODEL=$(cat /proc/device-tree/model 2>/dev/null)
echo $MODEL
SYSTEM_PROCESSOR=$(uname -m)

if [[ "$MODEL" == *"NVIDIA"* ]]; then
    echo "Detected NVIDIA Jetson platform"
    export ZSIBOT_CONFIG_ROOT=$PWD/config
    export ROS_DOMAIN_ID=24
    export LD_LIBRARY_PATH=$PWD/../install/robots_dog_msgs/lib:/opt/ros/humble/lib:/opt/robot/robots_dog_msgs/lib:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/opt/TensorRT/lib:/usr/lib/aarch64-linux-gnu/nvidia/:$PWD/lib:$LD_LIBRARY_PATH
    export ZSIBOT_CONFIG_ROOT=/opt/robot/perception/install/perception/config
    export ZSIBOT_CACHE=/home/jszr/.zsibot/cache
    
    ./bin/perception_jobs ./config/perception/perception_job/perception_job.yaml 
else
    source /opt/ros/humble/setup.bash
    export ZSIBOT_CONFIG_ROOT=$PWD/config
    ./bin/rs_driver &
    ./bin/perception_jobs ./config/perception/perception_job/perception_job.yaml 
fi