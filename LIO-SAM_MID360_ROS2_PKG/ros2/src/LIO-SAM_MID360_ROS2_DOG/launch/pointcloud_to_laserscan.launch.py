import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 尝试从标准Python包导入配置参数
import sys, os
sys.path.insert(0, os.path.dirname(__file__))   # 让解释器能找到同级模块
try:
    from lio_sam_global_config import *
except ImportError:
    # 如果导入失败，设置默认值
    DEFAULT_USE_SIM_TIME = True

def generate_launch_description():
    # 获取包的share目录
    share_dir = get_package_share_directory('lio_sam')
    
    
    
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
            'min_height': -1.0,           # 最小高度（过滤掉地面以下的点，调整为更紧的范围）
            'max_height': 1.5,            # 最大高度（过滤掉较高的点，限制在地面附近）
            'angle_min': -3.14159,        # -180度
            'angle_max': 3.14159,         # 180度
            'angle_increment': 0.0087,   # 激光扫描的角度增量（约0.25度，提高分辨率）
            'scan_time': 0.1,             # 扫描时间
            
            'range_min': 0.3,             # 增加最小距离，过滤掉近距离噪声 (原0.8)
            'range_max': 20.0,             # 减少最大距离，避免远距离噪声影响 (原10.0)
            'use_inf': False,              # 是否使用无穷大值（布尔类型，不使用引号）
            
            'inf_epsilon': 1000.0,           # 无穷大值的替代值
            
            # # QoS设置，确保与rviz2订阅者兼容
            # 'qos_overrides./scan.publisher.reliability': 'reliable',
            # 'qos_overrides./scan.publisher.depth': 10,
            
            # 其他参数
            'use_sim_time': DEFAULT_USE_SIM_TIME,
            # 使用当前时间戳而不是原始时间戳，避免时间戳不匹配问题
            # 'use_latest_timestamp': 'True',
            # 设置目标坐标系为odom，确保laserscan保持水平，不随baselink倾斜
            'target_frame': 'base_link',
            
            # 确保laserscan投影到水平面，避免倾斜影响
            'concurrency_level': 1,       # 处理并发级别
        }],
        output='screen'
    )
    
    return LaunchDescription([
        pointcloud_to_laserscan_node,
    ])