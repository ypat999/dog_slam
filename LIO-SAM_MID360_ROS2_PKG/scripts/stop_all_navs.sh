#!/bin/bash
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


ps -ef | awk '/rosbridge/ {print $2}' | xargs -r kill -9
ps -ef | awk '/publisher/ {print $2}' | xargs -r kill -9
ps -ef | awk '/rclcpp/ {print $2}' | xargs -r kill -9
# ros2 daemon stop 
# ros2 daemon start