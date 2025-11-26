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
            NAV2_DEFAULT_WEB_SCRIPT_PATH,
            )
    except ImportError:
        # 如果导入失败，使用默认值  
        print(  "Warning: Failed to import glo    bal_config, using default values")
        NAV2_DEFAULT_WEB_SCRIPT_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh'


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
        declare_localization_cmd,
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