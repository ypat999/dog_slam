#!/bin/bash

# FAST-LIO节点重启脚本

echo "正在重启FAST-LIO节点..."

# 方法1：通过生命周期管理器重启
echo "方法1：通过生命周期管理器重启"
ros2 lifecycle set /fastlio_mapping shutdown
sleep 2
ros2 lifecycle set /fastlio_mapping configure
sleep 1
ros2 lifecycle set /fastlio_mapping activate

echo "FAST-LIO节点重启完成！"

# 检查节点状态
ros2 lifecycle get /fastlio_mapping