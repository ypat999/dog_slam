#!/bin/bash

# 设备路径
DEVICE="/dev/video31"

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

# 设备存在后执行的命令
echo "设备已准备好，可以执行后续操作。"

v4l2-ctl -d $DEVICE --set-fmt-video=width=1920,height=1080,pixelformat=NV12
v4l2-ctl -d /dev/v4l-subdev2 -c horizontal_flip=1 -c vertical_flip=1

sleep 3

# video_stream -l rtsp://127.0.0.1:8554/back -d $DEVICE -r 1 -b 2000000
gst-launch-1.0 v4l2src device=$DEVICE io-mode=4 do-timestamp=true ! capsfilter caps="video/x-raw(memory:DMABuf),width=1920,height=1080,framerate=25/1" ! mpph264enc gop=10 rc-mode=1 bps=2000000 ! rtspclientsink location=rtsp://127.0.0.1:8554/back
