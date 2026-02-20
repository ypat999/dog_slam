#!/bin/bash
echo "===== ROS2 统一导航启动脚本 ====="
WORKSPACE_DIR="/home/robot/rkbot"

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

ros2 launch zg_double_lidar double_lidar.launch.py
