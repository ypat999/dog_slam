import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launches the full Cartographer 3D SLAM pipeline with bag playback and visualization."""
    
    # 获取包路径
    pkg_share = get_package_share_directory('slam_offline')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_path = LaunchConfiguration('bag_path')
    cartographer_config_dir = "/public/github/dog_slam/src/slam_offline/config/"  # 使用我们自己的配置文件目录
    configuration_basename =  "cartographer_3d_with_imu.lua" # 使用启用IMU的配置文件
    rviz_config_dir = "/public/github/dog_slam/src/slam_offline/config/"

    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (bag) time'
        ),
        
        DeclareLaunchArgument(
            'bag_path',
            default_value='/public/dataset/robot/livox_record/_cropped_sync/', # 使用裁切后的数据作为默认路径
            description='Path to the bag file to play (can be cropped data)'
        ),

        # # RViz2 可视化
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', '/public/github/dog_slam/src/slam_offline/config/cartographer.rviz'],
        #     output='screen'
        # ),
        # 静态变换：map-> odom 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            output='screen',
            arguments=['0', '0', '0', '0',  '0', '0','map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # # 静态变换：odom -> base_link (机器人初始位置，无旋转)
        Node(
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_lidar_tilt',
            output='screen',
            arguments=['0', '0', '0', '0',  '-0.4236', '0','base_link', 'livox_frame'],
            # arguments=['0', '0', '0', '0',  '0', '0','base_link', 'livox_frame'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Cartographer 3D SLAM 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-use_imu_data', 'true',
                       '-imu_sampling_ratio', '1.0',
                       '-rangefinder_sampling_ratio', '1.0',
                       '-max_submaps_to_keep', '5',  # 保持更多子图
                       '-submap_resolution', '0.5',  # 子图分辨率
                       '-enable_timing_output', 'true',  # 启用时间输出调试
                       '-enable_loop_closure', 'true',  # 启用闭环检测
                       '-optimize_every_n_nodes', '80',  # 优化频率
                       '-global_sampling_ratio', '0.003',  # 全局采样比例
                       '-constraint_builder_sampling_ratio', '0.05',  # 约束构建器采样比例
                       ],
            remappings=[
                ('/points2', '/livox/lidar'),  # 使用实际的激光雷达话题
                ('/imu', '/livox/imu'),  # 使用实际的IMU话题
            ],
        ),

        # Cartographer占用网格发布器 - 优化参数以避免纹理问题
        # 添加延迟启动，确保Cartographer节点先启动
        TimerAction(
            period=5.0,  # 延迟5秒启动
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='occupancy_grid_node',
                    name='cartographer_occupancy_grid_node',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=[
                        '-resolution', resolution, 
                        '-publish_period_sec', publish_period_sec,
                        '-min_probability', '0.1',        # 最小概率阈值
                        '-max_probability', '0.9',        # 最大概率阈值
                        '-submap_query_batch_size', '10'   # 子图查询批次大小
                    ]
                )
            ]
        ),

        # Bag 数据播放，添加QoS配置覆盖、开始时间和时钟参数，慢放100倍
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--qos-profile-overrides-path', '/public/dataset/robot/reliability_override.yaml', '--clock', '--rate', '1.0'],
            name='rosbag_player',
            output='screen'
        ),
        
    ])
