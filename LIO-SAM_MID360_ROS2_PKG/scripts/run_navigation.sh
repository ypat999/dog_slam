#!/bin/bash
echo "===== ROS2 统一导航启动脚本 ====="
WORKSPACE_DIR="/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2"

# 设置导航模式
export MANUAL_BUILD_MAP=False
# export ROS_DOMAIN_ID=24
export ROS_LOCALHOST_ONLY=0
# echo "显示输出：$DISPLAY"
export PYTHONPATH="$PYTHONPATH:/home/ztl/.local/lib/python3.10/site-packages"

# 支持的LIO算法列表
SUPPORTED_ALGORITHMS=("fast_lio" "lio_sam" "point_lio" "super_lio" "no_lio")

# 检查参数
if [ $# -eq 0 ]; then
    echo "用法: $0 <slam_algorithm>"
    echo "支持的算法: ${SUPPORTED_ALGORITHMS[*]}"
    echo "示例: $0 fast_lio"
    echo "       $0 lio_sam"
    exit 1
fi

SLAM_ALGORITHM=$1

# 验证算法是否支持
if [[ ! " ${SUPPORTED_ALGORITHMS[@]} " =~ " ${SLAM_ALGORITHM} " ]]; then
    echo "错误: 不支持的SLAM算法 '$SLAM_ALGORITHM'"
    echo "支持的算法: ${SUPPORTED_ALGORITHMS[*]}"
    exit 1
fi

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


REMOTE_IP="192.168.168.100"
REMOTE_USER="robot"
REMOTE_PASS="1"

echo "正在启动远程 zg_pointcloud 服务..."
sshpass -p "$REMOTE_PASS" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_IP" "echo '$REMOTE_PASS' | sudo -S systemctl start zg_pointcloud"
if [ $? -eq 0 ]; then
    echo "远程 zg_pointcloud 服务已启动"
else
    echo "警告: 远程 zg_pointcloud 服务启动失败"
fi

echo "启动导航..."
echo "SLAM算法: $SLAM_ALGORITHM"
echo "MANUAL_BUILD_MAP=$MANUAL_BUILD_MAP"
export SLAM_ALGORITHM=$SLAM_ALGORITHM
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py
