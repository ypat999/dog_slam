from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import sys
import os

# 获取当前launch文件所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 导入全局配置
from ament_index_python.packages import get_package_share_directory
import sys, os

# 添加global_config包的路径到Python路径
try:
    import sys
    global_config_path = get_package_share_directory('global_config')
    if global_config_path not in sys.path:
        sys.path.insert(0, global_config_path)
    from global_config import (
        DEFAULT_BAG_PATH,
        DEFAULT_RELIABILITY_OVERRIDE,
        DEFAULT_USE_SIM_TIME,
        DEFAULT_USE_SIM_TIME_STRING,
        NAV2_DEFAULT_MAP_FILE,
        NAV2_DEFAULT_WEB_SCRIPT_PATH,
        NAV2_DEFAULT_BT_XML_PATH,
        NAV2_DEFAULT_USE_SIM_TIME,
        NAV2_DEFAULT_USE_SIM_TIME_STRING
    )
except ImportError:
    # 如果导入失败，使用默认值
    print("Warning: Failed to import global_config, using default values")
    DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
    DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
    DEFAULT_USE_SIM_TIME = True
    DEFAULT_USE_SIM_TIME_STRING = 'true'
    
    NAV2_DEFAULT_MAP_FILE = "/home/ztl/slam_data/grid_map/map.yaml"
    NAV2_DEFAULT_WEB_SCRIPT_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh'
    NAV2_DEFAULT_BT_XML_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/bt_straight_then_rotate.xml'
    NAV2_DEFAULT_USE_SIM_TIME = True
    NAV2_DEFAULT_USE_SIM_TIME_STRING = 'true'


def generate_launch_description():
    ns = LaunchConfiguration('ns', default='/')  # 默认 robot1
    # 定义启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    
    # 声明nav2相关参数
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=NAV2_DEFAULT_MAP_FILE,
        description='Full path to map file to load'
    )
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(current_dir, '../config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='amcl',
        description='Localization backend to use: amcl or slam_toolbox'
    )
    
    # fast-lio配置参数
    fast_lio_package_dir = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(fast_lio_package_dir, 'config')
    
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path for fast_lio'
    )
    
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file for fast_lio'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 集成模式：直接调用fast_lio的mapping.launch.py
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_lio_package_dir, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_path': LaunchConfiguration('config_path'),
            'config_file': LaunchConfiguration('config_file'),
            'offline': 'false',
            'rviz': 'false'  # 导航模式下不启动rviz，避免与可能的rviz导航配置冲突
        }.items()
    )
    
    nav2_and_web_actions = []
    
    # 根据 localization 参数选择包含哪一个 nav2 启动文件（amcl 或 slam_toolbox）
    # For AMCL we use a dedicated local launch that starts map_server + amcl
    # and forwards the main params file. This avoids the need for a second
    # YAML file and lets the nodes read their sections from nav2_params.yaml.
    nav2_amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            current_dir, 'nav2_amcl.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'True',
            'use_respawn': 'False',
            'log_level': 'info'
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'amcl'"]))
    )

    nav2_and_web_actions.append(nav2_amcl_include)
    
    # 调用web控制脚本
    nav2_and_web_actions.append(ExecuteProcess(
        cmd=['bash', NAV2_DEFAULT_WEB_SCRIPT_PATH],
        output='screen',
        shell=False
    ))
    
    # 创建并返回完整的launch description
    launch_actions = [
        PushRosNamespace(ns),
        # 1. 声明所有启动参数
        declare_use_sim_time_cmd,
        declare_map_file_cmd,
        declare_localization_cmd,
        declare_nav2_params_file_cmd,
        declare_config_path_cmd,
        declare_config_file_cmd,
        # 2. 启动主要组件
        fast_lio_launch
    ]
    
    # 3. 添加Nav2和Web控制动作（延迟5秒启动）
    if nav2_and_web_actions:
        delayed_nav2_launch = TimerAction(
            period=5.0,  # 延迟5秒启动Nav2，确保fast_lio已初始化
            actions=nav2_and_web_actions
        )
        launch_actions.append(delayed_nav2_launch)
    
    return LaunchDescription(launch_actions)