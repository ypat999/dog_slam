from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """发布静态TF变换"""
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        
        # 静态变换：map-> odom 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 静态变换：odom -> base_link (机器人初始位置)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_odom_base',
            output='screen',
            arguments=['0', '0', '0', '0', '-0.4236', '0', 'odom', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 静态变换：base_link -> livox_frame (LiDAR倾斜补偿)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_lidar_tilt',
            output='screen',
            arguments=['0', '0', '0', '0', '-0.4236', '0', 'base_link', 'livox_frame'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])