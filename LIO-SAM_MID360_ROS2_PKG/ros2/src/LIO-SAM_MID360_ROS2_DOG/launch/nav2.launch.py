from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription

# 尝试从标准Python包导入配置参数
import sys, os
sys.path.insert(0, os.path.dirname(__file__))   # 让解释器能找到同级模块
try:
    from lio_sam_global_config import (
        DEFAULT_USE_SIM_TIME,
        DEFAULT_MAP_FILE,
        BUILD_MAP
    )
    CONFIG_IMPORTED = True
except ImportError:
    # 如果导入失败，设置默认值
    CONFIG_IMPORTED = False
    DEFAULT_USE_SIM_TIME = 'True'
    DEFAULT_MAP_FILE = os.path.expanduser('/home/ywj/projects/map_grid/map.yaml')
    BUILD_MAP = False  # 默认不使用建图模式

def generate_launch_description():
    # 地图与参数文件路径
    map_dir = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager', default='True')
    share_dir = get_package_share_directory('lio_sam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)
    

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=DEFAULT_MAP_FILE,
        description='Full path to map file to load')

    # 只有在非建图模式下才声明和使用参数文件
    if not BUILD_MAP:
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_lifecycle_manager_cmd = DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='True',
        description='Whether to use lifecycle manager to manage nav2 nodes')


    #缺少 map → odom 或 odom → base_link，在此发布
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        output='screen',
        parameters=[{            'use_sim_time': use_sim_time        }],
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'map', '--child-frame-id', 'odom'],
        remappings=[
               ('/odom', 'lio_sam/mapping/odometry'),  # 将内部 /odom 映射到 /lio_sam/mapping/odometry
               ('/scan', '/lio_sam/scan')
           ])
        
    # 发布 odom → base_link 变换
    static_transform_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_base_link',
        output='screen',
        parameters=[{            'use_sim_time': use_sim_time        }],
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'odom', '--child-frame-id', 'base_link'])


    
    # 启动nav2控制器和规划器节点（使用lifecycle管理）
    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'True',  # 禁用自动启动，由lifecycle_manager管理
            'use_composition': 'True',
            'use_respawn': 'False',
            'container_name': 'nav2_container',
            'namespace': '',
            'use_namespace': 'False',
            'log_level': 'INFO',
            'use_map_server': 'True',
            'map_server_required': 'True',
            'use_amcl': 'True', 
            'amcl_required': 'True'}.items())

    # 网页控制界面节点（ROS2 bridge + Websocket）
    # 这里用 rosbridge + web_video_server 组合
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'port': 9090},
            {'default_call_service_timeout': 5.0},  # 设置服务调用超时为5.0秒
            {'call_services_in_new_thread': True},  # 在新线程中调用服务
            {'send_action_goals_in_new_thread': True}  # 在新线程中发送动作目标
        ]
    )


    # 创建启动描述
    launch_actions = [
        declare_map_cmd,
        declare_use_lifecycle_manager_cmd,
    ]
    
    # 只有在非建图模式下才添加nav2相关节点和参数
    if not BUILD_MAP:
        launch_actions.append(declare_params_file_cmd)
        launch_actions.append(nav2_nodes)
        launch_actions.append(rosbridge_websocket)
    
    return LaunchDescription(launch_actions)
