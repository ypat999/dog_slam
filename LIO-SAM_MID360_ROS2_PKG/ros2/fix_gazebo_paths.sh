#!/bin/bash

# 修复Gazebo路径问题
# 设置ROS_PACKAGE_PATH以便Gazebo能解析package://URI
echo "设置ROS2和Gazebo环境变量..."

# 设置ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/ros2_livox_simulation:$ROS_PACKAGE_PATH

# 设置Gazebo模型路径
export GAZEBO_MODEL_PATH=/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/ros2_livox_simulation:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/ros2_livox_simulation:$GAZEBO_RESOURCE_PATH

# 验证路径设置
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
echo "GAZEBO_RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"

# 检查mid360.stl文件是否存在
if [ -f "/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/ros2_livox_simulation/meshes/mid360.stl" ]; then
    echo "✓ mid360.stl文件存在"
else
    echo "✗ mid360.stl文件不存在"
fi

echo "环境变量设置完成！"

# 重新运行仿真
echo "启动仿真..."
ros2 launch ros2_livox_simulation livox_simulation.launch.py