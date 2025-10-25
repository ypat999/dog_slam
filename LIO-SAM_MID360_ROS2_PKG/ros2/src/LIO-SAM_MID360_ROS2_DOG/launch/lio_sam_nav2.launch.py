from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# 尝试从标准Python包导入配置参数
import sys, os
sys.path.insert(0, os.path.dirname(__file__))   # 让解释器能找到同级模块
try:
    from lio_sam_global_config import (
        DEFAULT_USE_SIM_TIME,
        DEFAULT_MAP_FILE,
        DEFAULT_WEB_SCRIPT_PATH,
        BUILD_MAP
    )
    CONFIG_IMPORTED = True
except ImportError:
    # 如果导入失败，设置默认值
    CONFIG_IMPORTED = False
    DEFAULT_USE_SIM_TIME = 'True'
    DEFAULT_MAP_FILE = os.path.expanduser('/home/ywj/projects/map_grid/map.yaml')
    DEFAULT_WEB_SCRIPT_PATH = '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/web/run_web.sh'
    BUILD_MAP = False  # 默认不使用建图模式


def generate_launch_description():

    ns = LaunchConfiguration('ns', default='/')   # 默认 robot1

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
    
    # 建图模式下启动octomap server
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {'resolution': 0.1},
            {'frame_id': 'odom'},
            {'sensor_model_type': 'Pointcloud'},
            {'pointcloud_topic': '/lio_sam/mapping/cloud_registered'},
            {'max_range': 50.0},
            {'min_range': 0.1},
            {'occupancy_min_z': -1.0},
            {'occupancy_max_z': 1.0},
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(str(BUILD_MAP).lower())
    )
    
    # 导航模式下启动Nav2和web
    nav2_and_web_actions = []
    if not CONFIG_IMPORTED or not BUILD_MAP:
        # 延迟启动Nav2的launch文件
        nav2_and_web_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                current_dir, 'nav2.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': nav2_params_file
            }.items()
        ))
        
        # 延迟调用AMCL的全局定位服务
        nav2_and_web_actions.append(TimerAction(
            period=5.0,  # Nav2启动后再延迟5秒调用服务
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/reinitialize_global_localization', 'std_srvs/srv/Empty'],
                    output='screen',
                    shell=False
                )
            ]
        ))
        
        # 调用web控制脚本
        nav2_and_web_actions.append(ExecuteProcess(
            cmd=['bash', DEFAULT_WEB_SCRIPT_PATH],
            output='screen',
            shell=False
        ))
    
    # 根据模式创建适当的延迟启动动作
    if nav2_and_web_actions:
        delayed_nav2_launch = TimerAction(
            period=5.0,  # 延迟5秒启动Nav2，确保LIO-SAM已初始化
            actions=nav2_and_web_actions
        )
    
    # 创建并返回完整的launch description
    # 注意: 启动参数声明必须在使用它们的操作之前
    launch_actions = [
        PushRosNamespace(ns),
        # 1. 声明所有启动参数
        declare_map_file_cmd,
        declare_nav2_params_file_cmd,
        # 2. 启动主要组件
        lio_sam_launch
    ]
    
    # 3. 根据模式添加相应的节点
    if CONFIG_IMPORTED and BUILD_MAP:
        # 建图模式：添加octomap server
        launch_actions.append(octomap_server)
    elif 'delayed_nav2_launch' in locals():
        # 导航模式：添加nav2和web
        launch_actions.append(delayed_nav2_launch)
    
    return LaunchDescription(launch_actions)