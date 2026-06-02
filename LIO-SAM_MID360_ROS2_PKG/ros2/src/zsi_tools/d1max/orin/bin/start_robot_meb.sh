#!/bin/bash
source /opt/robot/robot_meb/install/setup.bash

LOG_DIR="/tmp/log/alg_data"
LOG_LEVEL="info"

if [ ! -d "${LOG_DIR}" ]; then
  mkdir -p "${LOG_DIR}"
fi

ros2 launch robot_meb meb_node.launch.py \
  log_dir:="${LOG_DIR}" \
  log_level:="${LOG_LEVEL}"
