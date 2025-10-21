import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的share目录
    share_dir = get_package_share_directory('lio_sam')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 参数声明
    params_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (bag) time'
    )
    
    # PointCloud to LaserScan 节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # ('/cloud_in', '/lio_sam/deskew/cloud_deskewed'),
            ('/cloud_in', '/lio_sam/mapping/cloud_registered_raw'),
            ('/scan', '/lio_sam/scan'),
        ],
        parameters=[{
            'transform_tolerance': 0.01,
            'min_height': -1.0,           # 最小高度（过滤掉地面以下的点）
            'max_height': 1.5,            # 最大高度（过滤掉较高的点）
            'angle_min': -3.14159,        # -180度
            'angle_max': 3.14159,         # 180度
            'angle_increment': 0.0087,    # 激光扫描的角度增量
            'scan_time': 0.1,             # 扫描时间
            
            'range_min': 0.8,             # 最小距离
            'range_max': 10.0,            # 最大距离
            'use_inf': False,              # 是否使用无穷大值（布尔类型，不使用引号）
            
            'inf_epsilon': 1000.0,           # 无穷大值的替代值
            
            # # QoS设置，确保与rviz2订阅者兼容
            # 'qos_overrides./scan.publisher.reliability': 'reliable',
            # 'qos_overrides./scan.publisher.depth': 10,
            
            # 其他参数
            'use_sim_time': use_sim_time,
            # 使用当前时间戳而不是原始时间戳，避免时间戳不匹配问题
            # 'use_latest_timestamp': 'True',
            # 设置目标坐标系为base_link，确保与静态变换匹配
            'target_frame': 'base_link'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        params_declare,
        pointcloud_to_laserscan_node,
    ])