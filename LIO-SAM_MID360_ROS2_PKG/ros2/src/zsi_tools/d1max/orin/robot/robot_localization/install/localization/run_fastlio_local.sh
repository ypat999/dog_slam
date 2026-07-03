#!/bin/bash

# 设置 Fastlio2 配置文件路径
export LOCALIZATION_YAML_DIR="/home/yuanxq/202509/0902/localization2_1_ws/install/localization/share/localization/config/localization_params_fastlio.yaml"

# 启动 Fastlio2 定位系统
ros2 launch localization fastlio_localization.launch.py 