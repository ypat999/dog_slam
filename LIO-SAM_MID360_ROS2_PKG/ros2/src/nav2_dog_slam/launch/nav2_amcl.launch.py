from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
import os, sys


def generate_launch_description():
    # 导入全局配置
    try:
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import DEFAULT_USE_SIM_TIME, NAV2_DEFAULT_MAP_FILE, NAV2_DEFAULT_PARAMS_FILE, BUILD_MAP, AUTO_BUILD_MAP
    except ImportError:
        # 如果导入失败，使用默认值  
        print("Warning: Failed to import global_config, using default values")
        DEFAULT_USE_SIM_TIME = True
        NAV2_DEFAULT_MAP_FILE = "/home/ztl/slam_data/grid_map/map.yaml"
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
        BUILD_MAP = False
        AUTO_BUILD_MAP = False


    # bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('nav2_dog_slam')
    launch_dir = os.path.join(bringup_dir, 'launch')

    use_sim_time = DEFAULT_USE_SIM_TIME
    params_file = NAV2_DEFAULT_PARAMS_FILE
    map_yaml_file = NAV2_DEFAULT_MAP_FILE
    autostart = False
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

    # amcl node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        prefix=['taskset -c 5,6'],   # 绑定 CPU 4
    )
    
    # lifecycle manager node to configure and activate map_server and amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['amcl']
        }]
    )

    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
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
            'use_respawn': 'False',  # 确保节点不会意外重启
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


    # add nodes
    if not BUILD_MAP and not AUTO_BUILD_MAP:
        ld.add_action(map_server_node)
        ld.add_action(lifecycle_manager_map_server)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(navigation_include)
    ld.add_action(rosbridge_websocket)  # 添加 rosbridge_websocket 节点

    return ld
