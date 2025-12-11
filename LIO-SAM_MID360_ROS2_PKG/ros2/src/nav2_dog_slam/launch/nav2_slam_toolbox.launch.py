from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
import os, sys


def generate_launch_description():
    try:
        global_config_path = os .path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            DEFAULT_USE_SIM_TIME,
            NAV2_DEFAULT_MAP_FILE,
            )
    except ImportError:
        # 如果导入失败，使用默认值  
        print(  "Warning: Failed to import global_config, using default values")
        DEFAULT_USE_SIM_TIME = True
        NAV2_DEFAULT_MAP_FILE = "/home/ztl/slam_data/grid_map/map.yaml"


    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    use_sim_time = DEFAULT_USE_SIM_TIME
    params_file = os.path.join(get_package_share_directory('nav2_dog_slam'), 'config', 'nav2_params.yaml')
    map_yaml_file = NAV2_DEFAULT_MAP_FILE
    autostart = True
    log_level = 'info'

    ld = LaunchDescription()

    # map_server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # slam toolbox node (替代 amcl node)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', '/scan'), 
            # ('/odom', '/lio_sam/mapping/odometry'),
            ('/odom', '/Odometry'),  # FAST-LIO 的 odometry 话题
            ('/initialpose', '/initialpose')  # Enable initial pose setting
        ],
        respawn=True,  # 启用自动重启，防止崩溃后系统停止运行
        respawn_delay=2.0  # 重启延迟2秒
    )
    
    # lifecycle manager node to configure and activate map_server and slam_toolbox
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server']
        }]
    )

    # include the rest of navigation stack
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'autostart': str(autostart),
            'params_file': params_file,
            'use_composition': 'False',  # 显式设置为false以确保costmap能正确发布
            'use_respawn': 'True',  # 允许节点重启
            'container_name': 'nav2_container',
            'log_level': log_level
        }.items()
    )
    
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

    # 使用定时器确保正确的启动顺序
    delayed_map_server = TimerAction(period=1.0, actions=[map_server_node])
    delayed_slam = TimerAction(period=2.0, actions=[slam_toolbox_node])
    delayed_lifecycle = TimerAction(period=3.0, actions=[lifecycle_manager])
    delayed_navigation = TimerAction(period=4.0, actions=[navigation_include])

    # add nodes
    ld.add_action(delayed_map_server)
    ld.add_action(delayed_slam)
    ld.add_action(delayed_lifecycle)
    ld.add_action(delayed_navigation)
    ld.add_action(rosbridge_websocket)  # 添加 rosbridge_websocket 节点

    return ld