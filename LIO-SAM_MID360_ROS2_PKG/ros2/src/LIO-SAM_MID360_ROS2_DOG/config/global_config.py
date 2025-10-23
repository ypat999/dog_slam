# LIO-SAM_MID360_ROS2_DOG全局配置参数
# 此文件作为标准Python模块，可以被其他模块导入使用

# 激光雷达在线/离线模式配置
ONLINE_LIDAR = True

# 默认配置参数
DEFAULT_USE_SIM_TIME = 'False'  # 是否使用仿真时间

# 默认数据集路径
DEFAULT_BAG_PATH = '/home/ywj/projects/dataset/robot/livox_record_new/'  # 使用裁切后的数据作为默认路径

# 默认可靠性覆盖文件路径
DEFAULT_RELIABILITY_OVERRIDE = '/home/ywj/projects/dataset/reliability_override.yaml'

# 默认LOAM数据保存目录
DEFAULT_LOAM_SAVE_DIR = '/home/ztl/slam_data/loam/'

# Nav2导航相关配置
DEFAULT_MAP_FILE = '/home/ywj/projects/map_grid/map.yaml'

# Web控制脚本路径
DEFAULT_WEB_SCRIPT_PATH = '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/web/run_web.sh'

# 坐标系名称配置
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_LINK_FRAME = 'base_link'
LIVOX_FRAME = 'livox_frame'
LIDAR_LINK_FRAME = 'lidar_link'

# 话题名称配置（可选，方便统一管理）
# LIDAR_TOPIC = '/livox/lidar'
# IMU_TOPIC = '/livox/imu'