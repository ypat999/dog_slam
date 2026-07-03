#! /bin/bash

. /opt/runtime/env.bash

source /opt/robot/robot-driver/install/setup.bash
ros2 launch uss_driver zsm_v4.launch.py
