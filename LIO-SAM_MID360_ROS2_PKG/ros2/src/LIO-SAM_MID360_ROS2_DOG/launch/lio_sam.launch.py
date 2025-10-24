import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
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

def generate_launch_description():
    ################### Livox LiDAR配置参数 ###################
    xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0    # 0-PointCloud2格式输出
    frame_id      = 'livox_frame'  # LiDAR坐标系名称
    # lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    default_user_config_path = os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json')
    user_config_path = LaunchConfiguration('user_config_path', default=default_user_config_path)

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    bag_path = DEFAULT_BAG_PATH
    reliability_file_path = DEFAULT_RELIABILITY_OVERRIDE

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'liosam_params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    #请另开窗口，source install/setup.sh后使用命令行录制，在launch内录制会缺少meta文件
    rosbag_record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/home/ztl/slam_data/livox_record_tilt/', 
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
        arguments=['0.0', '0.0', '0.0', '0.0', '0.5235987756', '0.0', 'base_link', 'livox_frame'],
        output='screen'
    )

    # base_link -> lidar_link (确保pointcloud_to_laserscan能正常工作的静态变换)
    static_transform_livox_to_lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_livox_to_lidar_link',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'livox_frame', 'lidar_link'],
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
    lio_sam_imuPreintegration_node = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen'
    )
    lio_sam_imageProjection_node = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen'
    )
    lio_sam_featureExtraction_node = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen'
    )
    lio_sam_mapOptimization_node = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[parameter_file, {'use_sim_time': DEFAULT_USE_SIM_TIME}],
        output='screen'
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',                   # 地图坐标系
            'sensor_model/max_range': 10.0,      # 最大感测距离
            'sensor_model/min_range': 0.8,       # 最小感测距离
            'sensor_model/insert_free_space': True,
            'resolution': 0.05,                  # OctoMap 分辨率（5cm）
            'occupancy_min_z': -0.5,             # 投影高度下限
            'occupancy_max_z': 0.5,              # 投影高度上限
            'publish_2d_map': True,               # 输出2D occupancy grid（布尔类型，不使用引号）
            'use_sim_time': DEFAULT_USE_SIM_TIME,
        }],
        remappings=[
            ('/cloud_in', '/lio_sam/mapping/cloud_registered')  # 输入点云
        ]
    )

        # 包含pointcloud_to_laserscan.launch.py
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'pointcloud_to_laserscan.launch.py')
        )
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

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


    launch_nodes.extend([
        static_transform_map_to_odom,  # 添加地图到里程计的静态变换
        static_transform_odom_to_base_link,  # 添加里程计到机器人基坐标系的静态变换
        # static_transform_base_to_livox,  # 添加机器人基坐标系到激光雷达的静态变换
        # static_transform_base_link_to_lidar_link,  # 添加base_link到lidar_link的静态变换
        robot_state_publisher_node,
        lio_sam_imuPreintegration_node,
        lio_sam_imageProjection_node,
        lio_sam_featureExtraction_node,
        lio_sam_mapOptimization_node,
        octomap_server_node,
        pointcloud_to_laserscan_launch,
        rviz2_node
    ])
    
    return LaunchDescription(launch_nodes)

