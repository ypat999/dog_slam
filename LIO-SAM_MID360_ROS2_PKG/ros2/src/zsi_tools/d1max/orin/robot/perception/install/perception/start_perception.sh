#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")

# 切换到脚本所在目录
cd "$SCRIPT_DIR"

MODEL=$(cat /proc/device-tree/model 2>/dev/null)
echo $MODEL

if [[ "$MODEL" == *"3588"* ]]; then
    echo "Detected RK3588 platform"
    PLATFORM="rk3588"
    SYSTEM_PROCESSOR=$(uname -m)

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${SYSTEM_PROCESSOR}-linux-gnu/lib:$PWD/lib

    export ZSIBOT_CONFIG_ROOT=$PWD/config
    ./bin/detection_2d_node config/perception/detection_2d.yaml

elif [[ "$MODEL" == *"NVIDIA"* ]]; then
    echo "Detected NVIDIA Jetson platform"
    PLATFORM="nvidia"
    SYSTEM_PROCESSOR=$(uname -m)
    export ROS_DOMAIN_ID=24
    export LD_LIBRARY_PATH=$PWD/../install/robots_dog_msgs/lib:/opt/ros/humble/lib:/opt/robot/robots_dog_msgs/lib:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/opt/TensorRT/lib:/usr/lib/aarch64-linux-gnu/nvidia/:$PWD/lib:$LD_LIBRARY_PATH

    export ZSIBOT_CONFIG_ROOT=$PWD/config
    ./bin/detection_2d_node config/perception/detection_2d.yaml
else
    source /opt/ros/humble/setup.bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/tensorrt/targets/${SYSTEM_PROCESSOR}-linux-gnu/lib:$PWD/lib

    export ZSIBOT_CONFIG_ROOT=$PWD/config
    ./bin/detection_2d_node config/perception/detection_2d.yaml
fi
