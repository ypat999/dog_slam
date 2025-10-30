#!/bin/bash
echo "===== ROS2 导航启动脚本 ====="
WORKSPACE_DIR="/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2"

# 设置导航模式
export BUILD_MAP=False
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0
# echo "显示输出：$DISPLAY"
export PYTHONPATH="$PYTHONPATH:/home/ztl/.local/lib/python3.10/site-packages"

# 检查是否已加载ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "加载ROS2环境..."
    source /opt/ros/humble/setup.bash

    if [ $? -ne 0 ]; then
        echo "错误: 无法加载ROS2环境"
        exit 1
    fi
fi

# 加载工作空间环境
echo "加载工作空间环境..."
source $WORKSPACE_DIR/install/setup.bash


echo "启动导航..."
echo "BUILD_MAP=$BUILD_MAP"
ros2 launch lio_sam lio_sam_nav2.launch.py ns:=/
