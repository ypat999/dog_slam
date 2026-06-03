#! /bin/bash
###
 # @Author: richie.li
 # @Date: 2025-12-04 13:42:45
 # @LastEditors: richie.li
 # @LastEditTime: 2025-12-04 13:43:25
### 

. /opt/runtime/env.bash

source /opt/robot/robot-driver/install/setup.bash
taskset -c 4,5 ros2 launch rslidar_sdk start.py
