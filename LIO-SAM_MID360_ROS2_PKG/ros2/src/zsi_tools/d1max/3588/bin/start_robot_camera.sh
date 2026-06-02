#!/bin/bash
# Device path
DEVICE="/dev/video22"

# Loop until the device exists
while true; do
    if [ -e "$DEVICE" ]; then
        echo "Device $DEVICE is available."
        break
    else
        echo "Device $DEVICE is not available, retrying in 1 second..."
        sleep 1
    fi
done

# Execute commands after the device is ready
echo "Device is ready. Executing follow-up operations."

v4l2-ctl -d $DEVICE --set-fmt-video=width=1920,height=1080,pixelformat=NV12
v4l2-ctl -d /dev/v4l-subdev7 -c horizontal_flip=1 -c vertical_flip=1


# Device path
DEVICE="/dev/video31"

# Loop until the device exists
while true; do
    if [ -e "$DEVICE" ]; then
        echo "Device $DEVICE is available."
        break
    else
        echo "Device $DEVICE is not available, retrying in 1 second..."
        sleep 1
    fi
done

# Execute commands after the device is ready
echo "Device is ready. Executing follow-up operations."

v4l2-ctl -d $DEVICE --set-fmt-video=width=1920,height=1080,pixelformat=NV12
v4l2-ctl -d /dev/v4l-subdev2 -c horizontal_flip=1 -c vertical_flip=1


# Start Rtsp Server Mediamtx
(
  cd /opt/runtime/bin || exit 1
  ./mediamtx ./mediamtx.yml > /tmp/mediamtx.log 2>&1 &
)

sleep 1

# Start robot_camera
source /opt/runtime/env.bash

ros2 launch robot_camera zsm.launch.py
