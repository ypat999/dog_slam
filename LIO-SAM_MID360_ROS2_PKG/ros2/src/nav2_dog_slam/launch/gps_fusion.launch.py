from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, sys


def generate_launch_description():
    # 导入全局配置
    try:
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import DEFAULT_USE_SIM_TIME
    except ImportError:
        print("Warning: Failed to import global_config, using default values")
        DEFAULT_USE_SIM_TIME = False

    use_sim_time = DEFAULT_USE_SIM_TIME
    
    # 获取包路径
    nav2_dog_slam_dir = get_package_share_directory('nav2_dog_slam')
    
    # 参数文件路径
    gps_ekf_params_file = os.path.join(nav2_dog_slam_dir, 'config', 'gps_ekf.yaml')
    navsat_transform_params_file = os.path.join(nav2_dog_slam_dir, 'config', 'navsat_transform.yaml')
    
    ld = LaunchDescription()
    
    # NavSat Transform节点 - GPS坐标转换
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_params_file],
        remappings=[
            ('/imu/data', '/imu/data'),  # IMU数据话题
            ('/gps/fix', '/gps/fix'),    # GPS原始数据话题
            ('/odometry/gps', '/odometry/gps'),  # 转换后的GPS里程计
            ('/odometry/filtered', '/odometry/filtered')  # EKF融合后的里程计
        ]
    )
    
    # EKF滤波器节点 - 传感器融合
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[gps_ekf_params_file],
        remappings=[
            ('/odometry/filtered', '/odometry/gps_fused'),  # GPS融合后的里程计
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # 生命周期管理器
    lifecycle_manager_gps = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_gps',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['navsat_transform_node', 'ekf_filter_node']
        }]
    )
    
    # 添加节点
    ld.add_action(navsat_transform_node)
    ld.add_action(ekf_filter_node)
    ld.add_action(lifecycle_manager_gps)
    
    return ld