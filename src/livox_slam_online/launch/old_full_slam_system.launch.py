import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取包目录
    pkg_livox_slam_online = get_package_share_directory('livox_slam_online')
    pkg_livox_ros_driver2 = get_package_share_directory('livox_ros_driver2')
    pkg_slam_offline = get_package_share_directory('slam_offline')
    
    # 配置文件路径
    mid360_config_path = os.path.join(pkg_livox_slam_online, 'config', 'mid360_config.json')
    cartographer_config_dir = os.path.join(pkg_livox_slam_online, 'config')
    cartographer_config_basename = 'livox_mid360_cartographer.lua'
    
    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Livox驱动节点
    livox_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_livox_ros_driver2, 'launch_ROS2', 'msg_MID360_launch.py')
        ),
        launch_arguments={
            'config_path': mid360_config_path,
            'user_config_path': mid360_config_path
        }.items()
    )
    
    # Cartographer节点
    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_offline, 'launch', 'cartographer_3d.launch.py')
        ),
        launch_arguments={
            'configuration_directory': cartographer_config_dir,
            'configuration_basename': cartographer_config_basename,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 数据录制节点
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', '/home/ywj/projects/dataset/robot/livox_record/', 
             '/livox/lidar', '/livox/imu', '/tf', '/tf_static',
             '/scan_matched_points2', '/submap_list', '/trajectory_node_list'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        livox_driver_node,
        cartographer_node,
        rosbag_record
    ])