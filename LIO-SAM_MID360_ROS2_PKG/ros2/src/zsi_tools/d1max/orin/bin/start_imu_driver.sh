#! /bin/bash

. /opt/runtime/env.bash

source /opt/robot/robot-driver/install/setup.bash
ros2 launch imu_driver imu_driver.launch.py
