#!/bin/bash
###
 # @Author: richie.li
 # @Date: 2025-10-15 22:57:17
 # @LastEditors: richie.li
 # @LastEditTime: 2025-12-05 20:03:43
### 

source /opt/runtime/env.bash

# 解析设备配置JSON文件，设置环境变量
PARSE_SCRIPT="/opt/runtime/bin/parse_device_json.sh"

# BOARD=$(${PARSE_SCRIPT} get board)
DEVICE_TYPE=$(${PARSE_SCRIPT} get type)

${PARSE_SCRIPT} print
echo ""

case "$DEVICE_TYPE" in
    "ZSM-1")
        echo "✓ 设备类型识别: ZSM-1"
        ros2 launch robot_remote zs_m1.launch.py
        ;;
    "ZSM-1F")
        echo "✓ 设备类型识别: ZSM-1F"
        ros2 launch robot_remote zs_m1f.launch.py
        ;;
    *)
        echo "✗ 未知的设备类型: $DEVICE_TYPE, 以默认方式启动 Robot Remote"
        ros2 launch robot_remote zs_m1.launch.py
        ;;
esac
