#!/usr/bin/env python3
"""
Livox LiDAR与Cartographer 3D建图集成启动文件
用于启动Livox LiDAR驱动和Cartographer 3D SLAM系统进行实时建图
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 获取包的共享目录
    livox_slam_online_dir = get_package_share_directory('livox_slam_online')
    # slam_offline_dir = get_package_share_directory('slam_offline') 
    
    ################### Livox LiDAR配置参数 ###################
    xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0    # 0-PointCloud2格式输出
    frame_id      = 'livox_frame'  # LiDAR坐标系名称
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'
    
    default_user_config_path = os.path.join(livox_slam_online_dir, 'config', 'mid360_config.json')
    user_config_path = LaunchConfiguration('user_config_path', default=default_user_config_path)
    
    # Cartographer配置参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cartographer_config_dir = os.path.join(livox_slam_online_dir, 'config')
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_3d_with_imu.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    
    # Livox LiDAR驱动参数
    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
        {"enable_imu_sync_time": True},
    ]
    
    print(f"Livox配置文件路径: {default_user_config_path}")
    print(f"Cartographer配置目录: {cartographer_config_dir}")
    
    # 创建LiDAR驱动节点
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    
    # # 创建静态变换节点，将livox_frame连接到base_link
    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']
    # )

    # 静态变换：map-> odom 
    static_transform_publisher_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0',  '0', '0','map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    ),

    # # 静态变换：odom -> base_link (机器人初始位置，无旋转)
    static_transform_publisher_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_base',
        output='screen',
        arguments=['0', '0', '0', '0',  '-0.4236', '0','odom', 'base_link'],
        # arguments=['0', '0', '0', '0',  '0', '0','odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    ),

    # 静态变换：base_link -> livox_frame (LiDAR倾斜补偿)
    # 补偿LiDAR 30度前倾：绕X轴旋转-30度（弧度约为-0.5236）
    # 参数顺序：x y z yaw pitch roll
    static_transform_publisher_lidar_tilt = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lidar_tilt',
        output='screen',
        arguments=['0', '0', '0', '0',  '-0.4236', '0','base_link', 'livox_frame'],
        # arguments=['0', '0', '0', '0',  '0', '0','base_link', 'livox_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    ),
    
    # 创建RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(livox_slam_online_dir, 'config', 'cartographer.rviz')]
    )
    
    # Cartographer 3D SLAM 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename], 
        remappings=[
            ('/points2', '/livox/lidar'),  # 重映射到Livox LiDAR的点云话题
            ('/imu', '/livox/imu'),  # 使用实际的IMU话题
        ],
    )
    
    # Cartographer占用网格发布器
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', '/public/dataset/robot/livox_record/', 
             '/livox/lidar', '/livox/imu', '/tf', '/tf_static',
             '/scan_matched_points2', '/submap_list', '/trajectory_node_list'],
        output='screen'
    )
    
    # 创建DeclareLaunchArgument
    declare_user_config_path = DeclareLaunchArgument(
        'user_config_path',
        default_value=default_user_config_path,
        description='Path to Livox user config json'
    )
    
    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='cartographer_3d_with_imu.lua',
        description='Cartographer configuration file basename'
    )
    
    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the occupancy grid map'
    )
    
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Publish period of the occupancy grid map'
    )
    
    # 返回Launch描述
    return LaunchDescription([
        declare_user_config_path,
        declare_configuration_basename,
        declare_resolution,
        declare_publish_period_sec,

        # 启动可视化
        # rviz_node,
        
        # 启动静态变换
        # static_transform_publisher,
        static_transform_publisher_map_odom,
        static_transform_publisher_odom_base,
        static_transform_publisher_lidar_tilt,
        
        # 启动LiDAR驱动
        livox_driver_node,
        
        # 启动Cartographer建图
        cartographer_node,
        occupancy_grid_node,
        
        # 启动rosbag记录
        rosbag_record
    ])