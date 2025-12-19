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


def generate_launch_description():
    try:
        global_config_path = os .path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            BUILD_MAP, BUILD_TOOL, AUTO_BUILD_MAP, RECORD_ONLY, ONLINE_LIDAR,
            NAV2_DEFAULT_WEB_SCRIPT_PATH, NAV2_DEFAULT_PARAMS_FILE, NAV2_DEFAULT_MAP_FILE,
            )
    except ImportError:
        # 如果导入失败，使用默认值  
        print(  "Warning: Failed to import glo    bal_config, using default values")
        BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        AUTO_BUILD_MAP = False
        RECORD_ONLY = False
        ONLINE_LIDAR = False
        NAV2_DEFAULT_WEB_SCRIPT_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh'
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
        NAV2_DEFAULT_MAP_FILE = '/home/ztl/slam_data/grid_map/map.yaml'


    ns = LaunchConfiguration('ns', default='/')  # 默认 robot1

    
    # 声明nav2相关参数
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='amcl',
        description='Localization backend to use: amcl or slam_toolbox'
    )
    
    # fast-lio配置参数
    fast_lio_package_dir = get_package_share_directory('fast_lio')

    

    # 集成模式：直接调用fast_lio的mapping.launch.py
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_lio_package_dir, 'launch', 'mapping.launch.py')),
    )
    
    nav2_and_web_actions = []
    
    # 根据 localization 参数选择包含哪一个 nav2 启动文件（amcl 或 slam_toolbox）
    # For AMCL we use a dedicated local launch that starts map_server + amcl
    # and forwards the main params file. This avoids the need for a second
    # YAML file and lets the nodes read their sections from nav2_params.yaml.
    nav2_amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            current_dir, 'nav2_amcl.launch.py')]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'amcl'"]))
    )

    nav2_slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            current_dir, 'nav2_slam_toolbox.launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': NAV2_DEFAULT_MAP_FILE,
            'params_file': NAV2_DEFAULT_PARAMS_FILE,
            'slam_toolbox_params': NAV2_DEFAULT_PARAMS_FILE
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'slam_toolbox'"]))
    )

    nav2_and_web_actions.append(nav2_amcl_include)
    nav2_and_web_actions.append(nav2_slam_include)
    
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
        declare_localization_cmd,
        # 2. 启动主要组件
        fast_lio_launch
    ]
    
    # 3. 添加Nav2和Web控制动作（延迟5秒启动）

    # 3. 根据模式添加相应的节点
    if  BUILD_MAP:
        # 建图模式：添加octomap server
        pass
    elif nav2_and_web_actions:
        # 导航模式：添加nav2和web
        delayed_nav2_launch = TimerAction(
            period=5.0,  # 延迟5秒启动Nav2，确保fast_lio已初始化
            actions=nav2_and_web_actions
        )
        launch_actions.append(delayed_nav2_launch)

    # 4. 如果AUTO_BUILD_MAP为True，延迟20秒启动explore_lite
    if AUTO_BUILD_MAP and not BUILD_MAP:
        explore_lite_package_dir = get_package_share_directory('explore_lite')
        explore_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                explore_lite_package_dir, 'launch', 'explore.launch.py')])
        )
        delayed_explore_launch = TimerAction(
            period=60.0,  # 延迟20秒启动explore_lite
            actions=[explore_launch]
        )
        launch_actions.append(delayed_explore_launch)

    return LaunchDescription(launch_actions)