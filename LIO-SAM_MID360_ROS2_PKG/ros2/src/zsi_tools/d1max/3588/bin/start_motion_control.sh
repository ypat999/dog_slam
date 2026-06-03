#!/bin/bash
###
 # @Author: richie.li
 # @Date: 2025-10-25 21:53:12
 # @LastEditors: richie.li
 # @LastEditTime: 2026-03-04 11:16:33
### 

echo "start motion control"

DEVICE="/dev/shm/joint_cmd"

# 循环检查设备是否存在
while true; do
    if [ -e "$DEVICE" ]; then
        echo "设备 $DEVICE 已存在。"
        break
    else
        echo "设备 $DEVICE 不存在，等待 1 秒后重试..."
        sleep 1
    fi
done

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

export LD_LIBRARY_PATH=/opt/export/mc/bin
export ROBOT_TYPE=ZGWS

PARSE_SCRIPT="/opt/runtime/bin/parse_device_json.sh"
DEVICE_TYPE=$(${PARSE_SCRIPT} get type)

case "$DEVICE_TYPE" in
    "ZSM-1")
        echo "✓ 设备类型识别: ZSM-1"
        export ROBOT_TYPE=ZGWS
        ;;
    "ZSM-1F")
        echo "✓ 设备类型识别: ZSM-1F"
        export ROBOT_TYPE=ZG
        ;;
    *)
        echo "✗ 未知的设备类型: $DEVICE_TYPE, 以默认方式启动 MC"
        export ROBOT_TYPE=ZGWS
        ;;
esac
echo ""

# cd /opt/export/mc/bin && ./mc_ctrl r
cd /opt/export/mc/bin && taskset -c 7 ./mc_ctrl r
