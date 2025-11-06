from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription

import sys, os
sys.path.insert(0, os.path.dirname(__file__))
try:
    from lio_sam_global_config import (
        DEFAULT_USE_SIM_TIME,
        DEFAULT_MAP_FILE,
        BUILD_MAP
    )
    CONFIG_IMPORTED = True
except ImportError:
    CONFIG_IMPORTED = False
    DEFAULT_USE_SIM_TIME = 'True'
    DEFAULT_MAP_FILE = os.path.expanduser('/home/ywj/projects/map_grid/map.yaml')
    BUILD_MAP = False


def generate_launch_description():
    map_dir = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam_toolbox_params = LaunchConfiguration('slam_toolbox_params')
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)

    share_dir = get_package_share_directory('lio_sam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=DEFAULT_MAP_FILE,
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_slam_toolbox_params_cmd = DeclareLaunchArgument(
        'slam_toolbox_params',
        default_value=os.path.join(share_dir, 'config', 'nav2_params.yaml'),
        description='Full path to slam_toolbox parameters file')

    # Instead of including the top-level bringup (which will start localization/amcl by default),
    # include the map_server and navigation brings separately and pass the no-amcl params file.
    # Some installations don't provide a packaged map_server_launch.py script at
    # /opt/ros/.../share/nav2_map_server/launch. To avoid filesystem-dependence,
    # start the map_server node directly here with the required parameters.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{ 'yaml_filename': map_dir, 'use_sim_time': use_sim_time }]
    )

    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'container_name': 'nav2_container',
            'namespace': '',
            'use_namespace': 'False',
            'log_level': 'INFO'
        }.items()
    )

    # Start slam_toolbox in localization/relocalization mode to provide pose estimates instead of AMCL
    # Note: do NOT pass the raw map yaml/pgm file as `map_file_name` here — that parameter is for
    # serialized pose-graph files. Instead let slam_toolbox subscribe to the map coming from
    # the map_server (/map topic). We will start nav2/map_server first, then start slam_toolbox
    # after a short delay so it can receive the /map topic.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox_localization',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {
                'use_sim_time': use_sim_time,
                # force-disable map publishing/updating to keep nav2 map_server as authoritative
                'map_update_interval': 0.0,
                'publish_occupancy_map': False,
                'use_map_saver': False
            }
        ],
        remappings=[('/scan', '/lio_sam/scan'), ('/odom', '/lio_sam/mapping/odometry')]
    )

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

    launch_actions = [
        declare_map_cmd,
        declare_params_file_cmd,
        declare_slam_toolbox_params_cmd,
        rosbridge_websocket,  # 添加 rosbridge_websocket 节点
    ]

    if not BUILD_MAP:
        # Start nav2 (map_server) first so /map is available, then start slam_toolbox after a short delay
        delayed_map_server = TimerAction(period=0.5, actions=[map_server_node])
        delayed_nav = TimerAction(period=1.0, actions=[navigation_include])
        delayed_slam = TimerAction(period=5.0, actions=[slam_toolbox_node])
        launch_actions.append(delayed_map_server)
        launch_actions.append(delayed_nav)
        launch_actions.append(delayed_slam)
        # Note: initial pose publisher removed; prefer remapping odom/scan so slam_toolbox
        # receives correct odometry and can compute odom pose. If needed, set initial
        # pose from an external tool (rviz) or re-add a publisher that matches your runtime.

    return LaunchDescription(launch_actions)
