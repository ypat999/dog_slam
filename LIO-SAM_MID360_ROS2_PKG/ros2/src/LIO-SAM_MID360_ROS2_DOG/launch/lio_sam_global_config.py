# LIO-SAM_MID360_ROS2_DOG全局配置参数
# 此文件作为标准Python模块，可以被其他模块导入使用
import os
import platform

# 获取当前主机名
current_machine = platform.node()
print(f"当前运行主机: {current_machine}")

# 定义主机特定的配置字典
config_by_machine = {
    'RK3588': {
        # RK3588主机配置
        'ONLINE_LIDAR': True,  # 通常RK3588是开发板，可能连接实际的激光雷达
        'DEFAULT_BAG_PATH': '/home/ztl/slam_data/livox_record_new/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ztl/slam_data/reliability_override.yaml',
        'DEFAULT_LOAM_SAVE_DIR': '/home/ztl/slam_data/loam/',
        'DEFAULT_MAP_FILE': "/home/ztl/slam_data/grid_map/map_offline.yaml",
        'DEFAULT_WEB_SCRIPT_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/web/run_web_150.sh'
    },
    'jqr001': {
        # jqr001主机配置
        'ONLINE_LIDAR': False,  # jqr001可能主要用于离线数据处理
        'DEFAULT_BAG_PATH': '/home/ywj/projects/dataset/robot/livox_record_new/',
        # 'DEFAULT_BAG_PATH': '/home/ywj/projects/dataset/robot/livox_record_tilt/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/projects/dataset/reliability_override.yaml',
        'DEFAULT_LOAM_SAVE_DIR': '/home/ywj/projects/LOAM/',
        'DEFAULT_MAP_FILE': '/home/ywj/projects/map_grid/map.yaml',
        'DEFAULT_WEB_SCRIPT_PATH': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/web/run_web_137.sh'
    }
}

# 默认配置（当主机名不在配置字典中时使用）
default_config = {
    'ONLINE_LIDAR': False,
    'DEFAULT_BAG_PATH': '/home/ztl/slam_data/livox_record_new/',
    'DEFAULT_RELIABILITY_OVERRIDE': '/home/ztl/slam_data/reliability_override.yaml',
    'DEFAULT_LOAM_SAVE_DIR': '/home/ztl/slam_data/loam/',
    'DEFAULT_MAP_FILE': "/home/ztl/slam_data/grid_map/map_offline.yaml",
    'DEFAULT_WEB_SCRIPT_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/web/run_web.sh'
}

# 根据当前主机名选择配置
selected_config = config_by_machine.get(current_machine, default_config)

# 将配置参数导出为模块变量
ONLINE_LIDAR = selected_config['ONLINE_LIDAR']

DEFAULT_USE_SIM_TIME = True
DEFAULT_USE_SIM_TIME_STRING = 'True'
if not ONLINE_LIDAR:
    DEFAULT_USE_SIM_TIME = False
    DEFAULT_USE_SIM_TIME_STRING = 'False'

DEFAULT_BAG_PATH = selected_config['DEFAULT_BAG_PATH']
DEFAULT_RELIABILITY_OVERRIDE = selected_config['DEFAULT_RELIABILITY_OVERRIDE']
DEFAULT_LOAM_SAVE_DIR = selected_config['DEFAULT_LOAM_SAVE_DIR']
DEFAULT_MAP_FILE = selected_config['DEFAULT_MAP_FILE']
DEFAULT_WEB_SCRIPT_PATH = selected_config['DEFAULT_WEB_SCRIPT_PATH']

# 坐标系名称配置
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_LINK_FRAME = 'base_link'
LIVOX_FRAME = 'livox_frame'
LIDAR_LINK_FRAME = 'lidar_link'

# 话题名称配置（可选，方便统一管理）
# LIDAR_TOPIC = '/livox/lidar'
# IMU_TOPIC = '/livox/imu'