#!/bin/bash
#
# ros2_record.sh
# -----------------
# 后台录制指定 ROS 2 topics，带磁盘空间监控和退出清理逻辑。
#
# 用法：
#   ./ros2_record.sh -t "topic1 topic2 ..." -o /path/to/output -s 2048 -m 2048
#
# 参数：
#   -t  topic 列表（空格分隔)
#   -o  输出目录
#   -s  单个 bag 文件最大大小（MB)
#   -m  最小剩余空间阈值（MB，默认 2048)
#

set -e

source /opt/runtime/env.bash

# ===== 默认参数 =====
MAX_SIZE_MB=2048
MIN_FREE_MB=2048
OUTPUT_DIR="/home/robot/rosbags"
TOPICS="/joint_shm_controller/joint_cmd_echo \
        /joint_shm_controller/joint_states \
        /joint_shm_controller/joint_sensor \
        /battery_controller/battery1 \
        /battery_controller/battery2 \
        /imu_shm_publisher/imu_central \
        "

# ===== 解析参数 =====
while getopts "t:o:s:m:" opt; do
  case $opt in
    t) TOPICS="$OPTARG" ;;
    o) OUTPUT_DIR="$OPTARG" ;;
    s) MAX_SIZE_MB="$OPTARG" ;;
    m) MIN_FREE_MB="$OPTARG" ;;
    *) echo "用法: $0 -t \"topic1 topic2 ...\" -o <output_dir> -s <size_mb> -m <min_free_mb>"; exit 1 ;;
  esac
done

if [ -z "$TOPICS" ]; then
  echo "❌ 错误：未指定 topic 列表 (-t)"
  exit 1
fi

mkdir -p "$OUTPUT_DIR"

# ===== 自动清理旧包，仅保留最后一个 =====
echo "🧹 清理旧 bag 文件，仅保留最新的一个..."
BAG_DIRS=($(ls -1dt ${OUTPUT_DIR}/bag_* 2>/dev/null || true))
if [ ${#BAG_DIRS[@]} -gt 1 ]; then
  for ((i=1; i<${#BAG_DIRS[@]}; i++)); do
    echo "   删除旧目录：${BAG_DIRS[$i]}"
    rm -rf "${BAG_DIRS[$i]}"
  done
fi

# ===== 时间戳命名 =====
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_DIR="${OUTPUT_DIR}/bag_${TIMESTAMP}"
LOG_FILE="${BAG_DIR}/rosbag_record.log"
# mkdir -p "$BAG_DIR"

echo "🟢 开始后台录制 ROS 2 topics："
echo "   Topics: $TOPICS"
echo "   输出目录: $BAG_DIR"
echo "   单文件最大大小: ${MAX_SIZE_MB} MB"
echo "   最小磁盘剩余: ${MIN_FREE_MB} MB"
echo

# ===== 启动 rosbag 录制（后台运行) =====
ros2 bag record \
  $TOPICS \
  --output "$BAG_DIR" \
  --max-bag-size $((MAX_SIZE_MB * 1024 * 1024)) \
  2>&1 &
  # > "$LOG_FILE" 2>&1 &

RECORD_PID=$!
echo "📸 ros2 bag record 已启动 (PID: $RECORD_PID)"
echo "日志文件: $LOG_FILE"
echo

# ===== 定义清理函数 =====
cleanup() {
  echo
  echo "🛑 收到退出信号，停止 ros2 bag record (PID $RECORD_PID)..."
  kill $RECORD_PID >/dev/null 2>&1 || true
  wait $RECORD_PID 2>/dev/null || true
  echo "✅ 已安全停止录制，日志: $LOG_FILE"
  exit 0
}

# ===== 捕获退出信号 =====
trap cleanup SIGINT SIGTERM EXIT

# ===== 磁盘监控循环 =====
MONITOR_INTERVAL=30  # 每 30 秒检测一次

echo "🚀 启动磁盘空间监控（间隔 ${MONITOR_INTERVAL}s)..."
while sleep $MONITOR_INTERVAL; do
  # 当前剩余空间（MB)
  FREE_MB=$(df -Pm "$OUTPUT_DIR" | awk 'NR==2 {print $4}')
  
  if [ "$FREE_MB" -lt "$MIN_FREE_MB" ]; then
      echo "⚠️  磁盘空间不足（剩余 ${FREE_MB} MB < 阈值 ${MIN_FREE_MB} MB)"
      echo "🛑 停止 ros2 bag record (PID $RECORD_PID)"
      kill $RECORD_PID >/dev/null 2>&1 || true
      wait $RECORD_PID 2>/dev/null || true
      echo "📦 录制已安全停止。"
      echo "$(date '+%Y-%m-%d %H:%M:%S') [WARN] Disk low (${FREE_MB}MB), stopped recording." >> "$LOG_FILE"
      exit 0
  fi
  
  # 检查 rosbag 是否还在运行
  if ! ps -p $RECORD_PID >/dev/null 2>&1; then
  echo "ℹ️ ros2 bag record 已退出，停止监控。"
  break
  fi
done

wait $RECORD_PID 2>/dev/null || true
echo "✅ 脚本结束。"
