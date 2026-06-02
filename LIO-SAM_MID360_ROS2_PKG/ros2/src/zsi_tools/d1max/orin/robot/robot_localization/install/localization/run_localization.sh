#! /bin/bash

export LOCALIZATION_YAML_DIR="/home/yuanxq/2026/202601/0129/localization_ws/install/localization/share/localization/config/localization_params_fastlio.yaml"
export CALIBRATION_FILE_PATH="/home/yuanxq/param/xg/calibration_results.yaml"

ros2 launch localization localization.launch.py

