#!/bin/bash

# Nav2导航启动脚本
# 支持在线SLAM、离线SLAM和纯导航模式

set -e

# 显示帮助信息
show_help() {
    echo "使用方法: $0 [模式] [选项]"
    echo ""
    echo "模式:"
    echo "  online     - 在线SLAM + 导航 (实时建图)"
    echo "  offline    - 离线SLAM + 导航 (使用bag文件)"
    echo "  nav_only   - 纯导航 (使用已有地图)"
    echo ""
    echo "选项:"
    echo "  -b, --bag PATH       bag文件路径 (offline模式)"
    echo "  -m, --map PATH       地图文件路径 (nav_only模式)"
    echo "  -s, --sim-time       使用仿真时间 (默认: true)"
    echo "  -i, --imu-offset     IMU时间偏移 (默认: -20.0)"
    echo "  -h, --help           显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 online                    # 在线SLAM+导航"
    echo "  $0 offline -b /path/to/bag/  # 离线SLAM+导航"
    echo "  $0 nav_only -m map.yaml      # 纯导航模式"
    exit 0
}

# 默认参数
MODE=""
BAG_PATH="/public/dataset/robot/livox_record/_cropped/"
MAP_FILE=""
USE_SIM_TIME="true"
IMU_OFFSET="-20.0"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        online|offline|nav_only)
            MODE=$1
            shift
            ;;
        -b|--bag)
            BAG_PATH="$2"
            shift 2
            ;;
        -m|--map)
            MAP_FILE="$2"
            shift 2
            ;;
        -s|--sim-time)
            USE_SIM_TIME="$2"
            shift 2
            ;;
        -i|--imu-offset)
            IMU_OFFSET="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            ;;
        *)
            echo "未知选项: $1"
            show_help
            ;;
    esac
done

# 检查模式是否指定
if [ -z "$MODE" ]; then
    echo "错误: 必须指定运行模式 (online/offline/nav_only)"
    show_help
fi

# 检查必需参数
if [ "$MODE" = "nav_only" ] && [ -z "$MAP_FILE" ]; then
    echo "错误: nav_only模式必须指定地图文件 (-m)"
    show_help
fi

echo "=========================================="
echo "Nav2 导航启动脚本"
echo "=========================================="
echo "模式: $MODE"
echo "仿真时间: $USE_SIM_TIME"
echo "IMU偏移: $IMU_OFFSET"

if [ "$MODE" = "offline" ]; then
    echo "Bag路径: $BAG_PATH"
elif [ "$MODE" = "nav_only" ]; then
    echo "地图文件: $MAP_FILE"
fi
echo "=========================================="

# 构建启动命令
LAUNCH_CMD="ros2 launch slam_offline slam_nav_bringup.launch.py"
LAUNCH_CMD="$LAUNCH_CMD mode:=$MODE"
LAUNCH_CMD="$LAUNCH_CMD use_sim_time:=$USE_SIM_TIME"
LAUNCH_CMD="$LAUNCH_CMD imu_time_offset:=$IMU_OFFSET"

if [ "$MODE" = "offline" ]; then
    LAUNCH_CMD="$LAUNCH_CMD bag_path:=$BAG_PATH"
elif [ "$MODE" = "nav_only" ]; then
    LAUNCH_CMD="$LAUNCH_CMD map_yaml_file:=$MAP_FILE"
fi

# 执行启动命令
echo "执行命令: $LAUNCH_CMD"
echo "=========================================="

# 检查工作空间
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 启动导航
exec $LAUNCH_CMD