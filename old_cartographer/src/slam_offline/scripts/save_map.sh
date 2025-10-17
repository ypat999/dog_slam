#!/bin/bash

# 保存地图脚本
# 用于保存当前SLAM创建的地图

# 检查参数
if [ $# -lt 1 ]; then
    echo "使用方法: $0 <地图文件名> [输出目录]"
    echo "示例: $0 my_map /public/github/dog_slam/maps"
    exit 1
fi

MAP_NAME=$1
OUTPUT_DIR=${2:-"/public/github/dog_slam/maps"}

# 创建输出目录
mkdir -p $OUTPUT_DIR

# 等待地图服务可用
echo "等待地图服务..."
until ros2 service list | grep -q "/map_server/map"; do
    sleep 1
done

# 保存地图
echo "保存地图到 $OUTPUT_DIR/$MAP_NAME.yaml"
ros2 service call /map_server/map nav2_msgs/srv/SaveMap "{map_url: '$OUTPUT_DIR/$MAP_NAME.yaml', map_mode: 'trinary', image_format: 'pgm', map_resolution: 0.05, free_thresh: 0.196, occupied_thresh: 0.65}"

# 检查保存结果
if [ $? -eq 0 ]; then
    echo "地图保存成功！"
    echo "地图文件: $OUTPUT_DIR/$MAP_NAME.yaml"
    echo "图像文件: $OUTPUT_DIR/$MAP_NAME.pgm"
else
    echo "地图保存失败！"
    exit 1
fi