import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import sys, os
import math

def deg_to_rad(deg):
    """将角度转换为弧度"""
    return str(deg * math.pi / 180.0)

def generate_launch_description():
    # 首先导入全局配置

    # 正确导入global_config包
    try:
        # 方法1：通过ROS2包路径导入
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            ONLINE_LIDAR, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
            DEFAULT_USE_SIM_TIME, MANUAL_BUILD_MAP, BUILD_TOOL, RECORD_ONLY,
            NAV2_DEFAULT_PARAMS_FILE, LIVOX_MID360_CONFIG, LIVOX_MID360_CONFIG_NO_TILT, DEFAULT_NAMESPACE
        )
    except ImportError as e:
        print(f"方法2导入global_config失败: {e}")
        # 如果导入失败，使用默认值
        ONLINE_LIDAR = True
        DEFAULT_BAG_PATH = '/home/cat/slam_data/livox_record_new/'
        DEFAULT_RELIABILITY_OVERRIDE = '/home/cat/slam_data/reliability_override.yaml'
        DEFAULT_USE_SIM_TIME = False
        MANUAL_BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        RECORD_ONLY = False
        NAV2_DEFAULT_PARAMS_FILE = '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/config/nav2_params_sim.yaml'
        LIVOX_MID360_CONFIG_NO_TILT = ''
    
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'gazebo_mid360.yaml')
    # rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')
    
    use_sim_time = DEFAULT_USE_SIM_TIME
    livox_config_path = LIVOX_MID360_CONFIG_NO_TILT
    lidar_mode = "ONLINE"
    if not ONLINE_LIDAR:
        lidar_mode = "OFFLINE"

    ld = LaunchDescription()

    declare_ns_arg = DeclareLaunchArgument(
        'ns',
        default_value=DEFAULT_NAMESPACE,
        description='Namespace for multi-robot support'
    )
    ns = LaunchConfiguration('ns')
    
    ns_map_frame = PythonExpression(["'map' if '", ns, "' == '' else str('", ns, "/map')"])
    ns_odom_frame = PythonExpression(["'odom' if '", ns, "' == '' else str('", ns, "/odom')"])
    ns_base_frame = PythonExpression(["'base_footprint' if '", ns, "' == '' else str('", ns, "/base_footprint')"])
    ns_world_frame = PythonExpression(["'world' if '", ns, "' == '' else str('", ns, "/world')"])
    ns_imu_frame = PythonExpression(["'imu' if '", ns, "' == '' else str('", ns, "/imu')"])
    ns_livox_frame = PythonExpression(["'livox_frame' if '", ns, "' == '' else str('", ns, "/livox_frame')"])
    ns_base_link_frame = PythonExpression(["'base_link' if '", ns, "' == '' else str('", ns, "/base_link')"])
    
    ld.add_action(declare_ns_arg)

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')
    ld.add_action(declare_rviz_arg)



    # 在线模式：Livox雷达驱动
    # livox_driver_node = Node(
    #     package='livox_ros_driver2',
    #     executable='livox_ros_driver2_node',
    #     name='livox_lidar_publisher',
    #     output='screen',
    #     parameters=[
    #         {"xfer_format": 1},
    #         {"multi_topic": 0},
    #         {"data_src": 0},
    #         {"publish_freq": 10.0},
    #         {"output_data_type": 0},
    #         {"frame_id": 'livox_frame'},
    #         {"user_config_path": livox_config_path},
    #         {"cmdline_input_bd_code": 'livox0000000001'},
    #     ],
    #     prefix=['taskset -c 4'],   # 绑定 CPU 4
    #     condition=IfCondition(PythonExpression("'" + lidar_mode + "' == 'ONLINE'"))
    # )

    # # 离线模式：rosbag播放
    # from launch.actions import ExecuteProcess
    # rosbag_player = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'play', DEFAULT_BAG_PATH, '--qos-profile-overrides-path', DEFAULT_RELIABILITY_OVERRIDE, '--clock', '--rate', '1.0'],
    #     name='rosbag_player',
    #     output='screen',
    #     prefix=['taskset -c 4'],   # 绑定 CPU 4
    #     condition=IfCondition(PythonExpression("'" + lidar_mode + "' == 'OFFLINE'"))
    # )

    # 根据模式选择启动相应的节点
    # ld.add_action(rosbag_player)
    # if lidar_mode == "ONLINE":
    #     ld.add_action(livox_driver_node)

    
    use_sim_time = DEFAULT_USE_SIM_TIME

    # 创建Super-LIO生命周期节点
    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[
            config_yaml, 
            {'use_sim_time': DEFAULT_USE_SIM_TIME},
            {'lio.output.tf_base_footprint_frame': ns_base_frame},
            {'lio.output.world_frame': ns_world_frame},
            {'lio.output.imu_frame': ns_imu_frame},
        ],
        prefix=['taskset -c 7'],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('/lio/odom', 'lio/odom'),
            ('/lio/imu/odom', 'lio/imu/odom'),
            ('/lio/robo/odom', 'lio/robo/odom'),
            ('/lio/path', 'lio/path'),
            ('/lio/cloud_world', 'lio/cloud_world'),
            ('/lio/body/cloud', 'lio/body/cloud'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    ld.add_action(super_lio_node)

    # 添加静态变换发布器
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_map_frame, ns_odom_frame],
        output='screen'
    )
    ld.add_action(static_transform_map_to_odom)

    static_transform_odom_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_world',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_odom_frame, ns_world_frame],
        output='screen'
    )
    ld.add_action(static_transform_odom_to_world)

    # world -> imu (里程计到机器人基坐标系的静态变换)
    static_transform_world_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_world_to_imu',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_world_frame, ns_imu_frame],
        output='screen'
    )
    ld.add_action(static_transform_world_to_imu)

    imu_to_livox_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_livox_frame_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0', '0.0', '0', '0.0', '0', ns_imu_frame, ns_livox_frame],
        output='screen'
    )
    ld.add_action(imu_to_livox_frame_tf)

    # livox_frame -> base_link (机器人基坐标系到雷达坐标系的静态变换)
    # 旋转角度：pitch = 30度 (0.5236弧度)
    livox_frame_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_to_base_link_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['-0.1', '0', '-0.1', '0', str(deg_to_rad(-30)), '0', 'mid360_robot/livox_frame/lidar', ns_base_link_frame],
        output='screen'
    )
    ld.add_action(livox_frame_to_base_link_tf)

    # world -> basefootprint (里程计到机器人基坐标系的静态变换)
    static_transform_world_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_world_to_base_footprint',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_world_frame, ns_base_frame],
        output='screen'
    )
    # ld.add_action(static_transform_world_to_base_footprint)

    # 根据模式添加相应的节点（按照LIO-SAM的逻辑）
    if RECORD_ONLY:
        # 仅录制模式：只启动雷达驱动
        return ld

    

    return ld