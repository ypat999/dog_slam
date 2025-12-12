#!/bin/bash

# 根据不同主机名确定web目录路径
case $(hostname) in
    "RK3588")
        WEB_DIR="/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web"
        ;;
    "jqr001")
        WEB_DIR="/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web"
        ;;
    "DESKTOP-4LS1SSN"|"DESKTOP-ypat")
        WEB_DIR="/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web"
        ;;
    *)
        # 默认路径
        WEB_DIR="/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web"
        ;;
esac

# 检查目录是否存在，如果不存在则使用默认目录
if [ ! -d "$WEB_DIR" ]; then
    echo "警告: 目录 $WEB_DIR 不存在，使用默认目录"
    WEB_DIR="/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web"
fi

# 进入web目录并启动HTTP服务器
echo "切换到目录: $WEB_DIR"
cd "$WEB_DIR"
echo "启动Web服务器，端口: 8083"
python3 -m http.server 8083