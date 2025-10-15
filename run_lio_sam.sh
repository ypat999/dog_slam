#!/bin/bash

# LIO-SAM for Livox 顶层快速启动脚本
# 用于快速启动LIO-SAM激光-惯性里程计

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== LIO-SAM for Livox 启动器 ==="
echo "工作空间: ${SCRIPT_DIR}"

# 检查工作空间
if [ ! -f "${SCRIPT_DIR}/src/lio_sam_livox/package.xml" ]; then
    echo "错误: 找不到lio_sam_livox包，请检查工作空间结构"
    exit 1
fi

cd "${SCRIPT_DIR}"

# 检查是否已编译
if [ ! -d "install/lio_sam_livox" ]; then
    echo "检测到lio_sam_livox包未编译，开始编译..."
    source /opt/ros/galactic/setup.bash
    colcon build --packages-select lio_sam_livox --symlink-install
fi

# 设置ROS环境
source /opt/ros/galactic/setup.bash
source install/setup.bash

echo "=== 启动LIO-SAM ==="
echo "启动命令: ros2 launch lio_sam_livox lio_sam_livox_launch.py $*"
echo "注意: 这是一个简化的LIO-SAM启动脚本，提供LiDAR驱动和基本可视化功能"
echo "如需完整的SLAM功能，请使用其他专门的SLAM包"

# 启动LIO-SAM（简化版本）
ros2 launch lio_sam_livox lio_sam_livox_launch.py "$@"