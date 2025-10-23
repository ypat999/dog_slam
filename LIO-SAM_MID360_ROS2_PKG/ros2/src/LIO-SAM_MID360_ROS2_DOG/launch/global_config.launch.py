from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 从标准Python模块导入全局配置参数
try:
    from LIO_SAM_MID360_ROS2_DOG.config.global_config import (
        ONLINE_LIDAR,
        DEFAULT_USE_SIM_TIME,
        DEFAULT_BAG_PATH,
        DEFAULT_RELIABILITY_OVERRIDE,
        DEFAULT_LOAM_SAVE_DIR
    )
except ImportError:
    # 如果导入失败，使用默认值
    ONLINE_LIDAR = True
    DEFAULT_USE_SIM_TIME = 'False'
    DEFAULT_BAG_PATH = '/home/ywj/projects/dataset/robot/livox_record_new/'
    DEFAULT_RELIABILITY_OVERRIDE = '/home/ywj/projects/dataset/reliability_override.yaml'
    DEFAULT_LOAM_SAVE_DIR = '/home/ztl/slam_data/loam/'

def generate_launch_description():
    """统一管理use_sim_time参数的launch文件"""

    
    # 定义use_sim_time参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=DEFAULT_USE_SIM_TIME,
        description='Use simulation (bag) time')

    bag_path_declare = DeclareLaunchArgument(
        'bag_path',
        default_value=DEFAULT_BAG_PATH, # 使用裁切后的数据作为默认路径
        description='Path to the bag file to play (can be cropped data)'
    )

    declare_reliability_override_cmd = DeclareLaunchArgument(
        'reliability_override',
        default_value=DEFAULT_RELIABILITY_OVERRIDE,
        description='Full path to reliability override file to load')

    declare_loam_save_dir_cmd = DeclareLaunchArgument(
        'loam_save_dir',
        default_value=DEFAULT_LOAM_SAVE_DIR,
        description='Full path to loam save directory')
    
    # 创建并返回launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        bag_path_declare,
        declare_reliability_override_cmd,
        declare_loam_save_dir_cmd,
    ])