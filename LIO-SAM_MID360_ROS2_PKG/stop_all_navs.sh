#!/bin/bash
echo "===== ROS2 导航停止脚本 ====="
pkill -f lio
pkill -f LIO
pkill -f run_web_150
pkill -f rosbridge
pkill -f publisher
pkill -f rclcpp
pkill -f 'http.server 8083'


ps -ef | awk '/rosbridge/ {print $2}' | xargs -r kill -9
ps -ef | awk '/publisher/ && {print $2}' | xargs -r kill -9
ps -ef | awk '/rclcpp/ {print $2}' | xargs -r kill -9
ros2 daemon stop 
ros2 daemon start


