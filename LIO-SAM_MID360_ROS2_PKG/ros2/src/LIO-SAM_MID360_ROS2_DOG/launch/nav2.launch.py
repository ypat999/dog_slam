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
        DEFAULT_MAP_FILE
    )
    CONFIG_IMPORTED = True
except ImportError:
    # 如果导入失败，设置默认值
    CONFIG_IMPORTED = False
    DEFAULT_USE_SIM_TIME = 'True'
    DEFAULT_MAP_FILE = os.path.expanduser('/home/ywj/projects/map_grid/map.yaml')

def generate_launch_description():
    # 地图与参数文件路径
    map_dir = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager', default='True')
    share_dir = get_package_share_directory('lio_sam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)
    
    # PointCloud to LaserScan launch文件
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            share_dir, 'launch', 'pointcloud_to_laserscan.launch.py')])
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=DEFAULT_MAP_FILE,
        description='Full path to map file to load')

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

    # Nav2 核心节点组
    # 注意：这里不再使用nav2_bringup的默认启动，而是单独启动各个节点
    # 这样可以更好地控制启动过程和参数
    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[params_file, {'yaml_filename': map_dir}],
    #     remappings=[
    #         ('/map', '/map'),
    #         ('/map_metadata', '/map_metadata')
    #     ])
    
    # 注意：不再手动定义AMCL节点，而是通过bringup_launch.py统一管理
    # 这样可以避免节点重复启动的问题
    
    # 移除静态变换发布器，让AMCL动态发布map->odom变换

    # map_align = Node(
    #     package='lio_sam',
    #     executable='map_align_node',
    #     name='map_align_node',
    #     output='screen'
    # )


    
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

    # Lifecycle Manager节点 - 统一管理nav2节点状态
    # 直接从配置文件读取参数，避免重复定义
    # lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_navigation',
    #     output='screen',
    #     parameters=[params_file],
    #     condition=IfCondition(use_lifecycle_manager)
    # )

    # # 节点启动顺序控制器 - 确保节点按正确顺序启动
    # delayed_amcl = TimerAction(
    #     period=2.0,  # 延迟2秒启动AMCL，确保map_server已就绪
    #     actions=[amcl]
    # )

    # delayed_nav2_nodes = TimerAction(
    #     period=5.0,  # 延迟5秒启动其他nav2节点，确保AMCL已初始化
    #     actions=[nav2_nodes]
    # )

    initial_pose_pub = TimerAction(
        period=8.0,  # 延迟3秒，等待amcl启动
        actions=[
            ExecuteProcess(
                cmd=[ "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{ \
                          header: { \
                            frame_id: 'map'\
                          }, \
                          pose: { \
                            pose: { \
                              position: {x: 0.0, y: 0.0, z: 0.0}, \
                              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} \
                            }, \
                            covariance: [0.25, 0, 0, 0, 0, 0, \
                                         0, 0.25, 0, 0, 0, 0, \
                                         0, 0, 0.0, 0, 0, 0, \
                                         0, 0, 0, 0.0685, 0, 0, \
                                         0, 0, 0, 0, 0.0685, 0, \
                                         0, 0, 0, 0, 0, 0.0685] \
                          } \
                        }' "
                    ], 
                shell=True
            )
        ]
    )


    # 返回 launch description
    return LaunchDescription([
        declare_map_cmd,
        declare_params_file_cmd,
        declare_use_lifecycle_manager_cmd,
        # static_transform_map_to_odom,  # 发布 map → odom 变换
        # static_transform_odom_to_base_link,  # 发布 odom → base_link 变换
        # map_server,  # 单独启动的map_server
        # delayed_amcl,  # 延迟启动的AMCL节点
        # delayed_nav2_nodes,  # 延迟启动的其他nav2节点
        # lifecycle_manager,  # 
        # amcl_node,
        nav2_nodes,
        # initial_pose_pub,  # 发布初始位姿
        rosbridge_websocket,
    ])
