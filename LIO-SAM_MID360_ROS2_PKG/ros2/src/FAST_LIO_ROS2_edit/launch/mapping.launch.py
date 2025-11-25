import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 首先导入全局配置
    from ament_index_python.packages import get_package_share_directory
    import sys, os

    # 正确导入global_config包
    try:
        # 方法1：通过ROS2包路径导入
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            FAST_LIO_MODE, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
            DEFAULT_USE_SIM_TIME_STRING, FAST_LIO_BASE_CODE_PATH
        )
    except ImportError as e:
        print(f"方法1导入global_config失败: {e}")
        try:
            # 方法2：直接通过相对路径导入
            current_dir = os.path.dirname(os.path.abspath(__file__))
            global_config_path = os.path.join(current_dir, '../../global_config')
            sys.path.insert(0, global_config_path)
            from global_config import (
                FAST_LIO_MODE, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
                DEFAULT_USE_SIM_TIME_STRING, FAST_LIO_BASE_CODE_PATH
            )
        except ImportError as e2:
            print(f"方法2导入global_config失败: {e2}")
            # 如果导入失败，使用默认值
            FAST_LIO_MODE = 'online'
            DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
            DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
            DEFAULT_USE_SIM_TIME_STRING = 'false'
            FAST_LIO_BASE_CODE_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/FAST_LIO_ROS2/'
    
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value=DEFAULT_USE_SIM_TIME_STRING,
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    # 离线模式相关参数
    offline_mode = LaunchConfiguration('offline_mode', default='true' if FAST_LIO_MODE == 'offline' else 'false')
    bag_path = LaunchConfiguration('bag_path', default=DEFAULT_BAG_PATH)
    reliability_file_path = LaunchConfiguration('reliability_file_path', default=DEFAULT_RELIABILITY_OVERRIDE)

    declare_offline_mode_cmd = DeclareLaunchArgument(
        'offline_mode', default_value='false',
        description='Enable offline mode with rosbag playback'
    )
    declare_bag_path_cmd = DeclareLaunchArgument(
        'bag_path', default_value='/home/ywj/projects/dataset/robot/livox_record_tilt_test2/',
        description='Path to rosbag file or directory'
    )
    declare_reliability_file_path_cmd = DeclareLaunchArgument(
        'reliability_file_path', default_value='/home/ywj/projects/dataset/reliability_override.yaml',
        description='Path to reliability override file'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    
    ld.add_action(declare_offline_mode_cmd)
    ld.add_action(declare_bag_path_cmd)
    ld.add_action(declare_reliability_file_path_cmd)

    # 离线模式：rosbag播放
    from launch.actions import ExecuteProcess
    rosbag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--qos-profile-overrides-path', LaunchConfiguration('reliability_file_path'), '--clock', '--rate', '1.0'],
        name='rosbag_player',
        output='screen',
        condition=IfCondition(LaunchConfiguration('offline_mode'))
    )

    livox_share_dir = get_package_share_directory('livox_ros_driver2')
    default_user_config_path = os.path.join(livox_share_dir, 'config', 'MID360_config.json')

    # 在线模式：Livox雷达驱动
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": 'livox_frame'},
            {"user_config_path": default_user_config_path},
            {"cmdline_input_bd_code": 'livox0000000001'},
            {"enable_imu_sync_time": True},
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('offline_mode'), "' == 'false'"])),
    )

    # 根据模式选择启动相应的节点
    ld.add_action(rosbag_player)
    ld.add_action(livox_driver_node)

    # 使用TimerAction添加延迟启动FAST-LIO节点，确保雷达数据就位
    from launch.actions import TimerAction
    ld.add_action(
        TimerAction(
            period=3.0,  # 延迟3秒启动FAST-LIO节点
            actions=[fast_lio_node]
        )
    )
    
    # ld.add_action(rviz_node)




    # PointCloud to LaserScan 节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # ('/cloud_in', '/lio_sam/deskew/cloud_deskewed'),
            ('/cloud_in', 'cloud_registered'),
            ('/scan', '/scan'),
        ],
        parameters=[{
            'transform_tolerance': 0.1,
            'min_height': -0.2,           # 最小高度（过滤掉地面以下的点，调整为更紧的范围）
            'max_height': 1.5,            # 最大高度（过滤掉较高的点，限制在地面附近）
            'angle_min': -3.1,        # -180度
            'angle_max': 3.1,         # 180度
            # 将角度增量精确设置为 (angle_max - angle_min) / (691 - 1)
            # 原始地图中的激光束数量为 691，实际转换产生 690 时会触发 slam_toolbox 的长度校验错误。
            # 使用精确值可以避免舍入导致的“expected 691 / got 690”问题。
            'angle_increment': 0.00869347338,
            'scan_time': 0.1,             # 扫描时间
            
            'range_min': 0.3,             # 增加最小距离，过滤掉近距离噪声 (原0.8)
            'range_max': 200.0,             # 减少最大距离，避免远距离噪声影响 (原10.0)
            'use_inf': False,              # 是否使用无穷大值（布尔类型，不使用引号）
            
            'inf_epsilon': 10000.0,           # 无穷大值的替代值
            
            # # QoS设置，确保与rviz2订阅者兼容
            # 'qos_overrides./scan.publisher.reliability': 'reliable',
            # 'qos_overrides./scan.publisher.depth': 10,
            
            # 其他参数
            'use_sim_time': use_sim_time,
            # 使用当前时间戳而不是原始时间戳，避免时间戳不匹配问题
            # 'use_latest_timestamp': 'True',
            # 设置目标坐标系为odom，确保laserscan保持水平，不随baselink倾斜
            'target_frame': 'base_link',
            'concurrency_level': 1,       # 处理并发级别
        }],
        output='screen'
    )
    ld.add_action(pointcloud_to_laserscan_node)

    # 修正TF变换链：确保所有坐标系变换方向正确
    # 正确的TF链：map → odom → camera_init → body → base_link → livox_frame
    
    # # 1. map到odom的静态变换（地图坐标系到里程计坐标系）
    # map_to_odom_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     output='screen'
    # )
    # ld.add_action(map_to_odom_tf)

    # # 2. odom到camera_init的静态变换（里程计坐标系到FAST-LIO初始坐标系）
    # odom_to_camera_init_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
    #     output='screen'
    # )
    # ld.add_action(odom_to_camera_init_tf)

    # 3. body到base_link的静态变换（FAST-LIO估计的机器人身体坐标系到机器人基座坐标系）
    # FAST-LIO发布camera_init到body的动态变换，这里设置body到base_link的静态变换
    base_link_to_lidar_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.1', '0', '-0.1', '0', '0', '0', 'lidar_link', 'base_link'],
        output='screen'
    )
    ld.add_action(base_link_to_lidar_link_tf)

    return ld