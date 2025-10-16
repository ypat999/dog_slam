import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """完整的SLAM+导航启动文件，支持在线和离线模式"""
    
    # 获取包路径
    pkg_share = get_package_share_directory('slam_offline')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')  # online, offline, nav_only
    bag_path = LaunchConfiguration('bag_path')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    imu_time_offset = LaunchConfiguration('imu_time_offset')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        
        DeclareLaunchArgument(
            'mode',
            default_value='offline',
            description='运行模式: online(在线SLAM+导航), offline(离线SLAM+导航), nav_only(仅导航，使用已有地图)'
        ),
        
        DeclareLaunchArgument(
            'bag_path',
            default_value='/home/ywj/projects/dataset/_cropped/',
            description='Bag文件路径（在线和离线模式使用）'
        ),
        
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='',
            description='地图YAML文件路径（仅导航模式使用）'
        ),
        
        DeclareLaunchArgument(
            'imu_time_offset',
            default_value='-20.0',
            description='IMU时间偏移'
        ),
        
        # 在线SLAM模式组
        GroupAction(
            actions=[
                # 启动在线SLAM
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'livox_slam_online.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'imu_time_offset': imu_time_offset
                    }.items()
                ),
                
                # 启动导航（使用在线SLAM的地图）
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'nav2_navigation.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'slam_mode': 'online',
                        'autostart': 'true'
                    }.items()
                ),
            ],
            condition=IfCondition(mode == 'online')
        ),
        
        # 离线SLAM模式组
        GroupAction(
            actions=[
                # 启动离线SLAM（带bag播放）
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'cartographer_3d.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'bag_path': bag_path,
                        'imu_time_offset': imu_time_offset
                    }.items()
                ),
                
                # 启动导航（使用离线SLAM的地图）
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'nav2_navigation.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'slam_mode': 'offline',
                        'autostart': 'true'
                    }.items()
                ),
            ],
            condition=IfCondition(mode == 'offline')
        ),
        
        # 仅导航模式组（使用已有地图）
        GroupAction(
            actions=[
                # 启动导航（使用已有地图）
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'nav2_navigation.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'slam_mode': 'none',
                        'map': map_yaml_file,
                        'autostart': 'true'
                    }.items()
                ),
                
                # 启动静态TF发布
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'static_tf.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items()
                ),
            ],
            condition=IfCondition(mode == 'nav_only')
        ),
    ])