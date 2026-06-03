#! /bin/bash

MAP="${1:-none}"

. /opt/runtime/env.bash

source /opt/robot/robot_nav/install/setup.bash
ros2 launch robot_nav2 navigation_bringup.launch.py platform:=REAL robot_type:=zg
