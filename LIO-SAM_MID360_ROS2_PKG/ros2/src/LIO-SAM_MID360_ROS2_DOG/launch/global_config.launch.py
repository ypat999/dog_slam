from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ONLINE_LIDAR = True

def generate_launch_description():
    """统一管理use_sim_time参数的launch文件"""

    
    # 定义use_sim_time参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (bag) time')

    bag_path_declare = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/ywj/projects/dataset/robot/livox_record_new/', # 使用裁切后的数据作为默认路径
        description='Path to the bag file to play (can be cropped data)'
    )

    declare_reliability_override_cmd = DeclareLaunchArgument(
        'reliability_override',
        default_value='/home/ywj/projects/dataset/reliability_override.yaml',
        description='Full path to reliability override file to load')

    declare_loam_save_dir_cmd = DeclareLaunchArgument(
        'loam_save_dir',
        # default_value='/home/ywj/projects/dataset/loam/',
        default_value='/home/ztl/slam_data/loam/',
        description='Full path to loam save directory')
    
    # 创建并返回launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        bag_path_declare,
        declare_reliability_override_cmd,
        declare_loam_save_dir_cmd,
    ])