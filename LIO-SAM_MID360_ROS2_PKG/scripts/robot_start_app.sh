#!/bin/sh
. /opt/runtime/env.bash

BASE_LOG_DIR="/home/robot/log"
MAX_LOG_DIRS=5

mkdir -p "${BASE_LOG_DIR}"

TIMESTAMP=$(date +"%Y%m%d-%H%M%S")
SESSION_NAME="robot-launch-log-${TIMESTAMP}"
SESSION_LOG_DIR="${BASE_LOG_DIR}/${SESSION_NAME}"
mkdir -p "${SESSION_LOG_DIR}"

ln -sfn "${SESSION_LOG_DIR}" "${ROBOT_LOG_DIR}/robot_launch_log"
LOG_FILE="${SESSION_LOG_DIR}/robot_launch.log"

ALL_DIRS=$(find "${BASE_LOG_DIR}" -maxdepth 1 -type d -name 'robot-launch-log*' -printf '%T@ %p\n' | sort -nr | awk '{print $2}')
CANDIDATES=$(printf "%s\n" "${ALL_DIRS}" | grep -vx -- "${SESSION_LOG_DIR}" || true)
DEL_DIRS=$(printf "%s\n" "${CANDIDATES}" | sed '/^$/d' | tail -n +$((MAX_LOG_DIRS+1)) || true)

if [ -n "${DEL_DIRS}" ]; then
  echo "🧹 Cleaning up old logs..." >> "${LOG_FILE}"
  printf "%s\n" "${DEL_DIRS}" | while IFS= read -r d; do
    [ -n "$d" ] && [ -d "$d" ] && { echo "   Removing: $d" >> "${LOG_FILE}"; rm -rf -- "$d"; }
  done
fi

# start zenoh router first
taskset -c 0,1,2,3,4,5,6 start_zenoh_router.sh > ${SESSION_LOG_DIR}/zenoh_router.log 2>&1 &
sleep 1

echo "robot_launch service started at: $(date)" >> "${LOG_FILE}"
taskset -c 0,1,2,3,4,5,6 robot-launch server &

while true; do
  echo "robot_launch running: $(date)" >> "${LOG_FILE}"
  sleep 3
done



