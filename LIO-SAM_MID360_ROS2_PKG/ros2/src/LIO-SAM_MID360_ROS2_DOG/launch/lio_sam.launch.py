import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
# 从标准Python包导入全局配置参数
import sys, os
sys.path.insert(0, os.path.dirname(__file__))   # 让解释器能找到同级模块
try:
    from lio_sam_global_config import *
except ImportError:
    # 如果导入失败，使用默认值
    print("ONLINE_LIDAR is None, set to True")
    ONLINE_LIDAR = True
    DEFAULT_USE_SIM_TIME = False
    DEFAULT_USE_SIM_TIME_STRING = 'FALSE'
    DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
    DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
    DEFAULT_LOAM_SAVE_DIR = '/home/ztl/slam_data/loam/'
    DEFAULT_MAP_FILE = "/home/ztl/slam_data/grid_map/map.yaml"
    USE_TILT_CONFIG = False
    BUILD_MAP = False  # 默认不使用建图模式
    BUILD_TOOL = 'octomap'

def generate_launch_description():
    ################### Livox LiDAR配置参数 ###################
    xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 20.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0    # 0-PointCloud2格式输出
    frame_id      = 'livox_frame'  # LiDAR坐标系名称
    # lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    share_dir = get_package_share_directory('lio_sam')
    livox_share_dir = get_package_share_directory('livox_ros_driver2')
    
    # 根据USE_TILT_CONFIG选择配置文件
    if 'USE_TILT_CONFIG' in globals() and USE_TILT_CONFIG:
        print("使用雷达倾斜配置文件")
        default_params_file = os.path.join(share_dir, 'config', 'liosam_params_tilt.yaml')
        xacro_path = os.path.join(share_dir, 'config', 'robot.urdf_tilt.xacro')
        # 选择MID360配置文件
        default_user_config_path = os.path.join(livox_share_dir, 'config', 'MID360_config_tilt.json')
    else:
        print("使用默认配置文件")
        default_params_file = os.path.join(share_dir, 'config', 'liosam_params.yaml')
        xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
        # 选择MID360配置文件
        default_user_config_path = os.path.join(livox_share_dir, 'config', 'MID360_config.json')
    
    user_config_path = LaunchConfiguration('user_config_path', default=default_user_config_path)
    
    parameter_file = LaunchConfiguration('params_file', default=default_params_file)
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    bag_path = DEFAULT_BAG_PATH
    reliability_file_path = DEFAULT_RELIABILITY_OVERRIDE

    # 参数声明部分不再需要，因为我们已经在上面设置了默认值
    # 保留声明但使用动态默认值
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    #请另开窗口，source install/setup.sh后使用命令行录制，在launch内录制会缺少meta文件
    rosbag_record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/home/ztl/slam_data/livox_record_tilt_new/', 
                 '/livox/lidar', '/livox/imu',],
            output='screen'
        )


    # Livox LiDAR驱动参数
    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        # {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
        {"enable_imu_sync_time": True},
    ]
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # 添加必要的静态变换发布器，建立坐标系变换树：map -> odom -> base_link -> livox_frame
    # map -> odom (地图到里程计的静态变换)
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )
    # odom -> base_link (里程计到机器人基坐标系的静态变换)
    static_transform_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_base_link',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'],
        output='screen'
    )
    # # base_link -> livox_frame (机器人基坐标系到激光雷达的静态变换)
    # # 激光雷达前倾30度 (转换为弧度约为0.5236)

    static_transform_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_livox',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        # arguments=['0.2', '0.0', '0.1', '0.0', '0.5235987756', '0.0', 'base_link', 'livox_frame'],
        arguments=['0.1', '0.0', '-0.3', '0.0', '0.0', '0.0', 'base_link', 'livox_frame'],
        output='screen'
    )

    # livox -> lidar_link (确保pointcloud_to_laserscan能正常工作的静态变换)
    static_transform_base_to_lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_lidar_link',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.1', '0.0', '-0.3', '0.0', '0.0', '0.0', 'base_link', 'lidar_link'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path]),
            'use_sim_time': DEFAULT_USE_SIM_TIME
        }]
    )
    # 创建带自动重启功能的LIO-SAM节点（使用Node类确保进程管理）
    lio_sam_imuPreintegration_node = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen',
        respawn=True,  # 启用自动重启
        respawn_delay=5.0  # 重启延迟5秒
    )
    lio_sam_imageProjection_node = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen',
        respawn=True,  # 启用自动重启
        respawn_delay=5.0  # 重启延迟5秒
    )
    lio_sam_featureExtraction_node = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen',
        respawn=True,  # 启用自动重启
        respawn_delay=5.0  # 重启延迟5秒
    )
    lio_sam_mapOptimization_node = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen',
        respawn=True,  # 启用自动重启
        respawn_delay=5.0  # 重启延迟5秒
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',                   # 地图坐标系
            'sensor_model/max_range': 100.0,      # 最大感测距离
            'sensor_model/min_range': 0.4,       # 最小感测距离
            'sensor_model/insert_free_space': True,
            'sensor_model/hit_prob': 0.9,        # 命中概率
            'sensor_model/miss_prob': 0.1,       # 未命中概率
            'resolution': 0.05,                  # OctoMap 分辨率（5cm）
            'filter_ground': True,              # 启用地面过滤
            'ground_filter/distance': 0.2,       # 地面过滤距离（20cm）
            'ground_filter/angle': 0.15,        # 地面角度（15度）
            'ground_filter/plane_distance': 0.07,  # 平面距离（7cm）
            'occupancy_min_z': -0.2,             # 投影高度下限
            'occupancy_max_z': 1.5,              # 投影高度上限
            'publish_2d_map': True,               # 输出2D occupancy grid（布尔类型，不使用引号）
            'use_sim_time': DEFAULT_USE_SIM_TIME,
        }],
        remappings=[
            ('/cloud_in', '/lio_sam/mapping/cloud_registered')  # 输入点云
        ]
    )

    # PointCloud to LaserScan 节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # ('/cloud_in', '/lio_sam/deskew/cloud_deskewed'),
            ('/cloud_in', '/lio_sam/mapping/cloud_registered'),
            ('/scan', '/lio_sam/scan'),
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
            'use_sim_time': DEFAULT_USE_SIM_TIME,
            # 使用当前时间戳而不是原始时间戳，避免时间戳不匹配问题
            # 'use_latest_timestamp': 'True',
            # 设置目标坐标系为odom，确保laserscan保持水平，不随baselink倾斜
            'target_frame': 'base_link',
            'concurrency_level': 1,       # 处理并发级别
        }],
        output='screen'
    )

    slam_toolbox_params = LaunchConfiguration('slam_toolbox_params')
    declare_slam_toolbox_params_cmd = DeclareLaunchArgument(
        'slam_toolbox_params',
        default_value=os.path.join(share_dir, 'config', 'nav2_params.yaml'),
        description='Full path to slam_toolbox parameters file')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox_localization',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {
                'use_sim_time': DEFAULT_USE_SIM_TIME,
                # force-disable map publishing/updating to keep nav2 map_server as authoritative
                'map_update_interval': 1.0,
                'publish_occupancy_map': True,
                'use_map_saver': True
            }
        ],
        remappings=[('/scan', '/lio_sam/scan'), ('/odom', '/lio_sam/mapping/odometry')]
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    if RECORD_ONLY:
        return LaunchDescription([livox_driver_node])

    launch_nodes = [params_declare]

    if ONLINE_LIDAR:
        launch_nodes.extend([
            livox_driver_node,
            # rosbag_record,
        ])
    else:
        launch_nodes.extend([
            # Bag 数据播放，添加QoS配置覆盖、开始时间和时钟参数
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_path, '--qos-profile-overrides-path', reliability_file_path, '--clock', '--rate', '1.0'],
                name='rosbag_player',
                output='screen'
            )
        ])


    # 添加坐标系转换节点
    launch_nodes.extend([
        static_transform_map_to_odom,  # 添加地图到里程计的静态变换
        static_transform_odom_to_base_link,  # 添加里程计到机器人基坐标系的静态变换
        static_transform_base_to_livox,  # 添加机器人基坐标系到激光雷达的静态变换
        static_transform_base_to_lidar_link,  # 添加livox到lidar_link的静态变换
    ])
    
    # 使用TimerAction添加延迟启动LIO-SAM核心节点，确保雷达数据就位
    # 延迟5秒启动LIO-SAM节点，给雷达足够的初始化时间
    lio_sam_nodes = [
        # robot_state_publisher_node,
        lio_sam_imuPreintegration_node,
        lio_sam_imageProjection_node,
        lio_sam_featureExtraction_node,
        lio_sam_mapOptimization_node
    ]
    
    launch_nodes.append(
        TimerAction(
            period=3.0,  # 延迟3秒启动LIO-SAM核心节点
            actions=lio_sam_nodes
        )
    )

        # 3. 根据模式添加相应的节点
    if BUILD_MAP:
        if BUILD_TOOL == 'slam_toolbox':
            launch_nodes.extend([
                pointcloud_to_laserscan_node,
                declare_slam_toolbox_params_cmd])
            launch_nodes.append(
                TimerAction(
                    period=50.0,  # 延迟5秒
                    actions=[slam_toolbox_node]
                )
            )    
        else:
            # 建图模式：添加octomap server
            launch_nodes.extend([
                # rviz2_node,
                # static_transform_map_to_odom,
                octomap_server_node])
    else:
        launch_nodes.extend([
            # rviz2_node,
            pointcloud_to_laserscan_node
        ])
    
    return LaunchDescription(launch_nodes)

