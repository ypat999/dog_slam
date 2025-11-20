#!/bin/bash

# 统一的导航启动脚本，支持切换FAST_LIO和LIO-SAM后端

# 默认参数
SLAM_BACKEND="lio_sam"  # 默认使用LIO-SAM
USE_SIM_TIME="false"
MAP_FILE="/home/ywj/projects/map_grid/map.yaml"
LOCALIZATION="amcl"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --slam-backend)
            SLAM_BACKEND="$2"
            shift 2
            ;;
        --use-sim-time)
            USE_SIM_TIME="$2"
            shift 2
            ;;
        --map-file)
            MAP_FILE="$2"
            shift 2
            ;;
        --localization)
            LOCALIZATION="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--slam-backend fast_lio|lio_sam] [--use-sim-time true|false] [--map-file PATH] [--localization amcl|slam_toolbox]"
            exit 1
            ;;
    esac
done

# 检查SLAM后端是否有效
if [[ "$SLAM_BACKEND" != "fast_lio" && "$SLAM_BACKEND" != "lio_sam" ]]; then
    echo "Error: Invalid SLAM backend. Use 'fast_lio' or 'lio_sam'"
    exit 1
fi

# 检查定位后端是否有效
if [[ "$LOCALIZATION" != "amcl" && "$LOCALIZATION" != "slam_toolbox" ]]; then
    echo "Error: Invalid localization backend. Use 'amcl' or 'slam_toolbox'"
    exit 1
fi

echo "Starting navigation with:"
echo "  SLAM Backend: $SLAM_BACKEND"
echo "  Localization: $LOCALIZATION"
echo "  Use Sim Time: $USE_SIM_TIME"
echo "  Map File: $MAP_FILE"

# 启动导航系统
ros2 launch nav2_dog_slam nav2_dog_slam.launch.py \
    slam_backend:="$SLAM_BACKEND" \
    localization:="$LOCALIZATION" \
    use_sim_time:="$USE_SIM_TIME" \
    map_file:="$MAP_FILE"
