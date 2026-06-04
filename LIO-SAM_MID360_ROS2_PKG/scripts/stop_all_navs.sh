#!/bin/bash

REMOTE_IP="192.168.168.100"
REMOTE_USER="robot"
REMOTE_PASS="1"

echo "正在停止远程 zg_pointcloud 服务..."
sshpass -p "$REMOTE_PASS" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_IP" "echo '$REMOTE_PASS' | sudo -S systemctl stop zg_pointcloud"
if [ $? -eq 0 ]; then
    echo "远程 zg_pointcloud 服务已停止"
else
    echo "警告: 远程 zg_pointcloud 服务停止失败"
fi

echo "退出本地服务"

pkill -f _lio
pkill -f lio_sam
pkill -f LIO_SAM
pkill -f run_web_
pkill -f rosbridge
pkill -f publisher
pkill -f rclcpp
pkill -f '8083'
pkill -f nav2
pkill -f gazebo
pkill -f gz
pkill -f pointcloud
pkill -f slam_toolbox

ps -ef | awk '/rosbridge/ {print $2}' | xargs -r kill
ps -ef | awk '/publisher/ {print $2}' | xargs -r kill
ps -ef | awk '/rclcpp/ {print $2}' | xargs -r kill

echo "sleep 3"

sleep 3

echo "强制退出本地服务"

pkill -9 -f _lio
pkill -9 -f lio_sam
pkill -9 -f LIO_SAM
pkill -9 -f run_web_
pkill -9 -f rosbridge
pkill -9 -f publisher
pkill -9 -f rclcpp
pkill -9 -f '8083'
pkill -9 -f nav2
pkill -9 -f gazebo
pkill -9 -f gz
pkill -9 -f pointcloud
pkill -9 -f slam_toolbox


ps -ef | awk '/rosbridge/ {print $2}' | xargs -r kill -9
ps -ef | awk '/publisher/ {print $2}' | xargs -r kill -9
ps -ef | awk '/rclcpp/ {print $2}' | xargs -r kill -9
# ros2 daemon stop 
# ros2 daemon start

echo "sleep 3"
sleep 3