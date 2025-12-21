#!/usr/bin/env python3

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

def generate_launch_description():
    # 导入全局配置
    try:
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            BUILD_MAP, BUILD_TOOL, AUTO_BUILD_MAP, RECORD_ONLY, ONLINE_LIDAR,
            NAV2_DEFAULT_WEB_SCRIPT_PATH, NAV2_DEFAULT_PARAMS_FILE, NAV2_DEFAULT_MAP_FILE,
            LIO_ALGORITHM  # 新增的算法选择参数
        )
    except ImportError:
        # 如果导入失败，使用默认值  
        print("Warning: Failed to import global_config, using default values")
        BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        AUTO_BUILD_MAP = False
        RECORD_ONLY = False
        ONLINE_LIDAR = False
        LIO_ALGORITHM = 'fast_lio'  # 默认使用fast_lio
        NAV2_DEFAULT_WEB_SCRIPT_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh'
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
        NAV2_DEFAULT_MAP_FILE = '/home/ztl/slam_data/grid_map/map.yaml'

    ns = LaunchConfiguration('ns', default='/')  # 默认 robot1

    # 声明启动参数
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='amcl',
        description='Localization backend to use: amcl or slam_toolbox'
    )
    
    declare_algorithm_cmd = DeclareLaunchArgument(
        'lio_algorithm',
        default_value=LIO_ALGORITHM,
        description='LIO algorithm to use: fast_lio, lio_sam, point_lio, faster_lio, dlio'
    )

    # 根据算法选择包含相应的launch文件
    # FAST-LIO
    fast_lio_package_dir = get_package_share_directory('fast_lio')
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_lio_package_dir, 'launch', 'mapping.launch.py')),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lio_algorithm'), "' == 'fast_lio'"]))
    )

    # LIO-SAM
    lio_sam_package_dir = get_package_share_directory('lio_sam')
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lio_sam_package_dir, 'launch', 'lio_sam.launch.py')),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lio_algorithm'), "' == 'lio_sam'"]))
    )

    # Point-LIO
    point_lio_package_dir = get_package_share_directory('point_lio')
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(point_lio_package_dir, 'launch_ros2', 'pointlio_launch.py')),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lio_algorithm'), "' == 'point_lio'"]))
    )

    # Faster-LIO
    faster_lio_package_dir = get_package_share_directory('faster_lio')
    faster_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(faster_lio_package_dir, 'launch_ros2', 'fasterlio_launch.py')),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lio_algorithm'), "' == 'faster_lio'"]))
    )

    # DLIO
    dlio_package_dir = get_package_share_directory('direct_lidar_inertial_odometry')
    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dlio_package_dir, 'launch_ros2', 'dlio_launch.py')),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lio_algorithm'), "' == 'dlio'"]))
    )

    nav2_and_web_actions = []

    # 根据 localization 参数选择包含哪一个 nav2 启动文件（amcl 或 slam_toolbox）
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
        declare_algorithm_cmd,
        # 2. 启动LIO算法（根据选择的算法，只有一个会实际启动）
        fast_lio_launch,
        lio_sam_launch,
        point_lio_launch,
        faster_lio_launch,
        dlio_launch
    ]

    # 3. 添加Nav2和Web控制动作（延迟5秒启动）
    if not BUILD_MAP and nav2_and_web_actions:
        delayed_nav2_launch = TimerAction(
            period=5.0,  # 延迟5秒启动Nav2，确保LIO算法已初始化
            actions=nav2_and_web_actions
        )
        launch_actions.append(delayed_nav2_launch)

    # 4. 如果AUTO_BUILD_MAP为True，延迟60秒启动explore_lite
    if AUTO_BUILD_MAP and not BUILD_MAP:
        explore_lite_package_dir = get_package_share_directory('explore_lite')
        explore_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                explore_lite_package_dir, 'launch', 'explore.launch.py')])
        )
        delayed_explore_launch = TimerAction(
            period=60.0,  # 延迟60秒启动explore_lite
            actions=[explore_launch]
        )
        launch_actions.append(delayed_explore_launch)

    return LaunchDescription(launch_actions)