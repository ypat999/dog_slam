#! /bin/bash

. /opt/runtime/env.bash

source /opt/robot/robot_localization/install/setup.bash

export LOCALIZATION_YAML_DIR="/opt/robot/robot_localization/install/localization/share/localization/config/localization_params_fastlio_zg.yaml"

ros2 launch localization localization_zg.launch.py
