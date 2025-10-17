from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 地图与参数文件路径
    map_dir = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    share_dir = get_package_share_directory('lio_sam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('/home/ywj/projects/map_grid/map.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Nav2 核心节点组
    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_file]),
        launch_arguments={
            'params-file': params_file,
            'map': map_dir,
            'use_sim_time': 'false'
            }.items()
    )
    # nav2_nodes = Node(
    #     package='nav2_bringup',
    #     executable='bringup_launch.py',
    #     output='screen',
    #     parameters=[{'use_sim_time': False}],
    #     arguments=['--ros-args', '--params-file', params_file, '--map', map_dir]
    # )
    # 网页控制界面节点（ROS2 bridge + Websocket）
    # 这里用 rosbridge + web_video_server 组合
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    # web_video_server = Node(
    #     package='web_video_server',
    #     executable='web_video_server',
    #     name='web_video_server',
    #     output='screen',
    #     parameters=[{'port': 8080}]
    # )

    # 返回 launch description
    return LaunchDescription([
        declare_map_cmd,
        declare_params_file_cmd,
        nav2_nodes,
        rosbridge_websocket,
        # web_video_server
    ])
