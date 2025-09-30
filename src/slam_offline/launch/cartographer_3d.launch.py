import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launches the full Cartographer 3D SLAM pipeline with bag playback and visualization."""
    
    # 获取包路径
    pkg_share = get_package_share_directory('my_cartographer_launch')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_path = LaunchConfiguration('bag_path')
    cartographer_config_dir = "/public/github/livox_ws/src/my_cartographer_launch/config/"  # 使用我们自己的配置文件目录
    configuration_basename = "cartographer_3d_with_imu.lua"  #"livox_mid360_cartographer.lua"  # 使用启用IMU的配置文件
    rviz_config_dir = "/public/github/livox_ws/src/my_cartographer_launch/config/"

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
            default_value='/public/dataset/robot/livox_record/', #'/public/dataset/robot/storage/',
            description='Path to the bag file to play'
        ),

        # RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/public/github/livox_ws/src/my_cartographer_launch/config/cartographer.rviz'],
            output='screen'
        ),

        # 创建静态变换节点，将livox_frame连接到base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']
        ),
        # # 静态变换发布器:  pandar_xt_32_0_lidar -> D455_1:imu (沿y轴顺时针旋转90度)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='pandar_xt_32_0_lidar_to_imu',
        #     # arguments=['0', '0', '0', '1.570796', '3.14159 ', '0', 'base_link', 'D455_1:imu']
        #     arguments=['0', '0', '0', '0', '0', '0', 'pandar_xt_32_0_lidar', 'D455_1:imu']
        # ),


        # # 静态变换发布器:  base_link -> pandar_xt_32_0_lidar (根据实际机器人结构设置正确的变换参数)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_lidar',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'pandar_xt_32_0_lidar']  # 调整旋转参数，修复坐标轴方向
        # ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_imu',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'D455_1:imu']  # 先假设IMU与base_link坐标系完全重合
        # ),


        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_link',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']  # 使用实际的安装位置参数
        # ),

        # Cartographer 3D SLAM 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       "-use_imu_data", "true"],
            remappings=[
                ('/points2', '/livox/lidar'),  # 使用实际的激光雷达话题
                ('/imu', '/livox/imu'),  # 使用实际的IMU话题
            ],
        ),

        # Cartographer占用网格发布器
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
        ),

        # Bag 数据播放，添加QoS配置覆盖和循环播放参数
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--qos-profile-overrides-path', '/public/dataset/robot/reliability_override.yaml', '--clock'],
            name='rosbag_player',
            output='screen'
        ),
        
    ])
