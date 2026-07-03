#!/bin/bash

# 获取当前脚本路径和基础目录
JSZR_BASE_DIR=/home/jszr/jszr_workspace

# 加载 ROS 环境
[ -f "/opt/ros/humble/setup.bash" ] || {
  echo "ROS setup.bash not found"
  exit 1
}
[ -f "${JSZR_BASE_DIR}/install/setup.bash" ] || {
  echo "Workspace setup.bash not found"
  exit 1
}

source /opt/ros/humble/setup.bash
source "${JSZR_BASE_DIR}/install/setup.bash"

# 设置日志路径
export ROS_LOG_DIR="${JSZR_BASE_DIR}/log"
mkdir -p "${ROS_LOG_DIR}"

# 定义每个标签的命令
discovery="fastdds discovery -i 0 -l 192.168.133.1 -p 20000"

# 检查 tmux 是否安装
command -v tmux >/dev/null 2>&1 || {
  echo "tmux is not installed. Please install it first."
  exit 1
}

# 创建新的 tmux 会话
SESSION_NAME="discovery_server"

function start() {
  # 创建新的会话并启动第一个命令
  tmux new-session -d -s "$SESSION_NAME" -n "discovery" bash -c "$discovery"

  # 设置默认窗口
  tmux select-window -t "$SESSION_NAME:0"

  # 设置 tmux 的历史缓冲区大小
  tmux set-option -w history-limit 100000
  # 启用鼠标滚动支持
  tmux set-option -g mouse on
  # 附加到会话
  tmux attach-session -t "$SESSION_NAME"
}

function stop {
  tmux kill-session -t "$SESSION_NAME"
}

# 检查是否已经存在同名的 tmux 会话
tmux has-session -t "$SESSION_NAME" 2>/dev/null
if [ $? != 0 ]; then
  start
else
  echo "A tmux session named '$SESSION_NAME' is already running."
  echo "Restarting..."
  stop && start
fi
