#/bin/bash

. /opt/runtime/env.bash
source /opt/robot/robot-driver/install/setup.bash

export LD_LIBRARY_PATH=/opt/robot/robot-driver/install/sixents_gps_driver/lib/:$LD_LIBRARY_PATH
ros2 launch sixents_gps_driver sixents_gps_driver.launch.py
