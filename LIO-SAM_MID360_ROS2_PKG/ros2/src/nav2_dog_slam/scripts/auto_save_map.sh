#!/bin/bash

# 自动保存地图脚本
# 每隔2分钟保存一次地图

SAVE_DIR="/home/ztl/slam_data"
MAP_PREFIX="map_auto"

# 创建保存目录（如果不存在）
mkdir -p "$SAVE_DIR"

echo "开始自动保存地图，每隔2分钟保存一次..."
echo "保存目录: $SAVE_DIR"
echo "按 Ctrl+C 停止"

count=0
while true; do
    count=$((count + 1))
    timestamp=$(date +"%Y%m%d_%H%M%S")
    map_name="${MAP_PREFIX}_${timestamp}"
    
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 第${count}次保存: ${map_name}"
    
    ros2 run nav2_map_server map_saver_cli -t /map -f "${SAVE_DIR}/${map_name}" --fmt png
    
    if [ $? -eq 0 ]; then
        echo "  -> 保存成功"
    else
        echo "  -> 保存失败"
    fi
    
    # 等待2分钟 (120秒)
    sleep 120
done
