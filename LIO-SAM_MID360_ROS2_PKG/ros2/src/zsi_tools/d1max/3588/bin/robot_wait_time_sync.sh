#!/bin/bash
leap_status=""
count=0
max_retries=25
while true; do
    # 获取 Leap 状态字段
    leap_status=$(chronyc tracking | grep -i "Leap status" | awk '{print $4}')
    if [ "$leap_status" = "Normal" ]; then
        echo "✅ 已同步（Leap status: $leap_status）"
        break
    else
        echo "⚠️  未同步（Leap status: $leap_status），等待同步中... [$((count+1))/$max_retries]"
	count=$((count + 1))
    fi

    # 超过最大次数则退出循环
    if [ $count -ge $max_retries ]; then
        echo "❌ 超过最大重试次数 ($max_retries)，仍未同步"
        systemctl stop gpsd.socket
        systemctl stop gpsd
	exit 0
    fi
    sleep 1
done
exit 0
