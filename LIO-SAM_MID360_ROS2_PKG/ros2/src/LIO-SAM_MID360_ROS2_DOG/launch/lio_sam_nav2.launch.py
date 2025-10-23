from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# 尝试从标准Python包导入配置参数
try:
    from LIO_SAM_MID360_ROS2_DOG.config.global_config import (
        DEFAULT_USE_SIM_TIME,
        DEFAULT_MAP_FILE,
        DEFAULT_WEB_SCRIPT_PATH
    )
    CONFIG_IMPORTED = True
except ImportError:
    # 如果导入失败，设置默认值
    CONFIG_IMPORTED = False
    DEFAULT_USE_SIM_TIME = 'True'
    DEFAULT_MAP_FILE = os.path.expanduser('/home/ywj/projects/map_grid/map.yaml')
    DEFAULT_WEB_SCRIPT_PATH = '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/web/run_web.sh'


def generate_launch_description():
    # 获取包的共享目录
    package_dir = get_package_share_directory('lio_sam')
    # 获取当前launch文件所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 定义启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)
    map_file = LaunchConfiguration('map_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=DEFAULT_MAP_FILE,
        description='Full path to map file to load')
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file')
    
    # 包含LIO-SAM的launch文件
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            current_dir, 'lio_sam.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 延迟启动Nav2的launch文件，确保LIO-SAM先完全启动
    delayed_nav2_launch = TimerAction(
        period=5.0,  # 延迟5秒启动Nav2，确保LIO-SAM已初始化
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    current_dir, 'nav2.launch.py')]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_file,
                    'params_file': nav2_params_file
                }.items()
            ),
            # 延迟调用AMCL的全局定位服务，确保Nav2完全启动
            TimerAction(
                period=5.0,  # Nav2启动后再延迟5秒调用服务
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/reinitialize_global_localization', 'std_srvs/srv/Empty'],
                        output='screen',
                        shell=False
                    )
                ]
            ),
            # 调用web控制脚本
            ExecuteProcess(
                cmd=['bash', DEFAULT_WEB_SCRIPT_PATH],
                output='screen',
                shell=False
            )
        ]
    )
    
    # 创建并返回完整的launch description
    # 注意: 启动参数声明必须在使用它们的操作之前
    return LaunchDescription([
        # 1. 声明所有启动参数
        declare_map_file_cmd,
        declare_nav2_params_file_cmd,
        # 2. 启动主要组件
        lio_sam_launch,  # 首先启动LIO-SAM
        delayed_nav2_launch  # 延迟启动Nav2并调用全局定位服务
    ])