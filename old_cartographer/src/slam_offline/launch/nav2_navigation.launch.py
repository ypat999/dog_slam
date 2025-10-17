import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """启动Nav2导航，支持在线和离线模式"""
    
    # 获取包路径
    pkg_share = get_package_share_directory('slam_offline')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    slam_mode = LaunchConfiguration('slam_mode')  # online, offline, none
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # 导航参数文件路径
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # 生命周期管理节点
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower'
    ]
    
    # 当slam_mode为none时，添加map_server和amcl
    slam_offline_condition = UnlessCondition(slam_mode == 'none')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='地图文件路径（YAML格式），当slam_mode为none时使用'
        ),
        
        DeclareLaunchArgument(
            'slam_mode',
            default_value='offline',
            description='SLAM模式: online(在线SLAM), offline(离线SLAM), none(使用已有地图)'
        ),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='自动启动导航栈'
        ),
        
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='使用组件化启动'
        ),
        
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='自动重启失败的节点'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别'
        ),
        
        # 地图服务器（仅在slam_mode为none时启动）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file
            }],
            condition=UnlessCondition(slam_mode == 'none')
        ),
        
        # AMCL定位（仅在slam_mode为none时启动）
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file],
            condition=UnlessCondition(slam_mode == 'none')
        ),
        
        # 控制器服务器
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav')  # 避免与teleop冲突
            ]
        ),
        
        # 规划器服务器
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # 恢复服务器
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # BT导航器
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # 航点跟随器
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # 速度平滑器
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'),
                ('/cmd_vel_smoothed', '/cmd_vel')
            ]
        ),
        
        # 生命周期管理器
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes + ['map_server', 'amcl'] if slam_mode == 'none' else lifecycle_nodes
            }]
        ),
        
        # 静态TF发布器（map->odom，当使用已有地图时）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(slam_mode == 'none')
        ),
        
        # 激光扫描转换器（将PointCloud2转换为LaserScan）
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'min_height': -0.5,
                'max_height': 1.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00436,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan')
            ]
        ),
        
        # RViz2 导航界面
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'nav2.rviz')]
        ),
    ])