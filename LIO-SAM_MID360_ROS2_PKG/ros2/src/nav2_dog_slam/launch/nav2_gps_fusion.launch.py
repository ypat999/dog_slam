from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindExecutable, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
import os, sys


def generate_launch_description():
    # 导入全局配置
    try:
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import DEFAULT_USE_SIM_TIME, NAV2_DEFAULT_MAP_FILE, NAV2_DEFAULT_PARAMS_FILE, MANUAL_BUILD_MAP, AUTO_BUILD_MAP, SLAM_ALGORITHM
    except ImportError:
        # 如果导入失败，使用默认值  
        print("Warning: Failed to import global_config, using default values")
        DEFAULT_USE_SIM_TIME = True
        NAV2_DEFAULT_MAP_FILE = "/home/ztl/slam_data/grid_map/map.yaml"
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
        MANUAL_BUILD_MAP = False
        AUTO_BUILD_MAP = False
        SLAM_ALGORITHM = 'super_lio'  # 默认算法

    # 定义不同LIO算法的话题映射配置
    LIO_TOPIC_CONFIGS = {
        'fast_lio': {
            'pointcloud_topic': '/cloud_registered_body',
            'odom_topic': '/Odometry',
            'octomap_topic': '/cloud_registered',
            'target_frame': 'base_footprint'
        },
        'lio_sam': {
            'pointcloud_topic': '/lio_sam/mapping/cloud_registered_raw',
            'odom_topic': '/lio_sam/mapping/odometry',
            'octomap_topic': '/lio_sam/mapping/cloud_registered',
            'target_frame': 'base_footprint'
        },
        'point_lio': {
            'pointcloud_topic': '/cloud_registered_body',
            'odom_topic': '/Odometry',
            'octomap_topic': '/cloud_registeredy',
            'target_frame': 'base_footprint'
        },
        'super_lio': {
            'pointcloud_topic': '/lio/body/cloud',
            'odom_topic': '/lio/odom',
            'octomap_topic': '/lio/cloud_world',
            'target_frame': 'base_footprint'
        }
    }
    
    # 获取当前选择的LIO算法的话题配置
    lio_config = LIO_TOPIC_CONFIGS.get(SLAM_ALGORITHM, LIO_TOPIC_CONFIGS['fast_lio'])

    use_sim_time = DEFAULT_USE_SIM_TIME
    params_file = NAV2_DEFAULT_PARAMS_FILE
    map_yaml_file = NAV2_DEFAULT_MAP_FILE
    autostart = True
    log_level = 'info'

    # bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('nav2_dog_slam')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ld = LaunchDescription()

    # 根据SLAM_ALGORITHM参数选择启动不同的SLAM算法
    # FAST-LIO
    try:
        fast_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')]),
            launch_arguments={
                'use_sim_time': str(use_sim_time)
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'fast_lio'"]))
        )
        ld.add_action(fast_lio_launch)
    except Exception as e:
        print(f"Fast-LIO package not found: {e}")
    
    # Point-LIO
    try:
        point_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('point_lio'), 'launch', 'mapping_mid360.launch.py')]),
            launch_arguments={
                'use_sim_time': str(use_sim_time)
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'point_lio'"]))
        )
        ld.add_action(point_lio_launch)
    except Exception as e:
        print(f"Point-LIO package not found: {e}")
    
    # LIO-SAM
    try:
        lio_sam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('lio_sam'), 'launch', 'lio_sam.launch.py')]),
            launch_arguments={
                'use_sim_time': str(use_sim_time)
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'lio_sam'"]))
        )
        ld.add_action(lio_sam_launch)
    except Exception as e:
        print(f"LIO-SAM package not found: {e}")

    # Super-LIO
    try:
        super_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('super_lio'), 'launch', 'Livox_mid360.py')]),
            launch_arguments={
                'use_sim_time': str(use_sim_time)
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'super_lio'"]))
        )
        ld.add_action(super_lio_launch)
    except Exception as e:
        print(f"Super-LIO package not found: {e}")

    # PointCloud to LaserScan 节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('/cloud_in', lio_config['pointcloud_topic']),
            ('/scan', '/scan'),
        ],
        parameters=[{
            'transform_tolerance': 0.1,
            'min_height': -0.1,
            'max_height': 1.0,
            'angle_min': -3.1,
            'angle_max': 3.1,
            'angle_increment': 0.00869347338,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 100.0,
            'use_inf': False,
            'inf_epsilon': 1000.0,
            'use_sim_time': use_sim_time,
            'target_frame': lio_config['target_frame'],
            'concurrency_level': 1,
        }],
        output='screen',
        prefix=['taskset -c 5'],
    )
    ld.add_action(pointcloud_to_laserscan_node)

    # map_server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ],
        prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 4
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # GPS融合节点 - 使用robot_localization
    gps_ekf_params_file = os.path.join(bringup_dir, 'config', 'gps_ekf.yaml')
    navsat_transform_params_file = os.path.join(bringup_dir, 'config', 'navsat_transform.yaml')
    
    # GPS预处理节点 - 处理GPS数据质量问题
    gps_preprocessor_node = Node(
        package='nav2_dog_slam',
        executable='gps_preprocessor.py',
        name='gps_preprocessor',
        output='screen',
        parameters=[{
            'min_satellites': 4,
            'max_hdop': 2.0,
            'min_accuracy': 0.1,
            'status_threshold': 0
        }]
    )
    
    # NavSat Transform节点 - GPS坐标转换
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_params_file],
        remappings=[
            ('/imu/data', '/imu/data'),  # IMU数据话题
            ('/gps/fix', '/gps/fix_filtered'),  # 使用预处理后的GPS数据
            ('/odometry/gps', '/odometry/gps'),  # 转换后的GPS里程计
            ('/odometry/filtered', '/odometry/gps_fused')  # EKF融合后的里程计
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

    # amcl node - 使用GPS融合后的里程计作为输入
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/initialpose', '/initialpose'),
            ('/amcl_pose', '/amcl_pose'),
            ('/particle_cloud', '/particle_cloud'),
            ('/scan', '/scan')
        ],
        prefix=['taskset -c 5,6'],   # 绑定 CPU 4
    )
    
    # lifecycle manager node to configure and activate map_server and amcl
    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['amcl']
        }]
    )

    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server']
        }]
    )
    
    # GPS融合生命周期管理器
    lifecycle_manager_gps = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_gps',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['gps_preprocessor', 'navsat_transform_node', 'ekf_filter_node']
        }]
    )
    

    # include the rest of navigation stack
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'autostart': str(autostart),
            'params_file': params_file,
            'use_composition': 'False',  # 显式设置为false以确保costmap能正确发布
            'use_respawn': 'False',  # 确保节点不会意外重启
            'container_name': 'nav2_container',
            'log_level': log_level
        }.items()
    )
    
    # 网页控制界面节点（ROS2 bridge + Websocket）
    # 这里用 rosbridge + web_video_server 组合
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'port': 9090},
            {'default_call_service_timeout': 5.0},  # 设置服务调用超时为5.0秒
            {'call_services_in_new_thread': True},  # 在新线程中调用服务
            {'send_action_goals_in_new_thread': True}  # 在新线程中发送动作目标
        ]
    )


    # add nodes
    if not MANUAL_BUILD_MAP and not AUTO_BUILD_MAP:
        ld.add_action(map_server_node)
        ld.add_action(lifecycle_manager_map_server)
    
    # 添加GPS融合节点
    ld.add_action(gps_preprocessor_node)
    ld.add_action(navsat_transform_node)
    ld.add_action(ekf_filter_node)
    ld.add_action(lifecycle_manager_gps)
    
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_amcl)
    ld.add_action(navigation_include)
    ld.add_action(rosbridge_websocket)  # 添加 rosbridge_websocket 节点

    return ld