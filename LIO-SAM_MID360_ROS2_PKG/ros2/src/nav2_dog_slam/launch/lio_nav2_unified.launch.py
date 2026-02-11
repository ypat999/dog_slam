from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

# 导入全局配置
from ament_index_python.packages import get_package_share_directory
import sys, os

# 添加global_config包的路径到Python路径
try:
    global_config_path = get_package_share_directory('global_config')
    if global_config_path not in sys.path:
        sys.path.insert(0, global_config_path)
    from global_config import (
        MANUAL_BUILD_MAP, BUILD_TOOL, AUTO_BUILD_MAP, RECORD_ONLY, NAVIGATION_MODE, 
        ONLINE_LIDAR as ONLINE_LIDAR, 
        LIO_SAM_BASE_CODE_PATH as BASE_CODE_PATH, 
        DEFAULT_USE_SIM_TIME as DEFAULT_USE_SIM_TIME,
        DEFAULT_USE_SIM_TIME_STRING as DEFAULT_USE_SIM_TIME_STRING, 
        DEFAULT_BAG_PATH as DEFAULT_BAG_PATH,
        DEFAULT_RELIABILITY_OVERRIDE as DEFAULT_RELIABILITY_OVERRIDE, 
        LIO_SAM_DEFAULT_LOAM_SAVE_DIR as DEFAULT_LOAM_SAVE_DIR,
        NAV2_BASE_CODE_PATH, NAV2_DEFAULT_MAP_FILE, NAV2_DEFAULT_WEB_SCRIPT_PATH,
        NAV2_DEFAULT_BT_XML_PATH, NAV2_DEFAULT_PARAMS_FILE,
        DEFAULT_USE_SIM_TIME_STRING, MAP_FRAME, ODOM_FRAME, 
        BASE_LINK_FRAME, LIVOX_FRAME, SLAM_ALGORITHM
    )
except Exception as e:
    print(f"导入global_config失败: {e}")
    # 如果导入失败，使用默认值
    MANUAL_BUILD_MAP = False
    BUILD_TOOL = 'octomap_server'
    AUTO_BUILD_MAP = False
    RECORD_ONLY = False
    NAVIGATION_MODE = 'standalone'
    ONLINE_LIDAR = False
    BASE_CODE_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/'
    DEFAULT_USE_SIM_TIME = True
    DEFAULT_USE_SIM_TIME_STRING = 'true'
    DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
    DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
    DEFAULT_LOAM_SAVE_DIR = '/home/ztl/slam_data/loam/'
    NAV2_BASE_CODE_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/'
    NAV2_DEFAULT_MAP_FILE = '/home/ztl/slam_data/grid_map/map.yaml'
    NAV2_DEFAULT_WEB_SCRIPT_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh'
    NAV2_DEFAULT_BT_XML_PATH = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
    NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
    MAP_FRAME = 'map'
    ODOM_FRAME = 'odom'
    BASE_LINK_FRAME = 'base_link' 
    LIVOX_FRAME = 'livox_frame'
    SLAM_ALGORITHM = 'fast_lio'  # 默认算法

# 获取当前launch文件所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 定义不同LIO算法的话题映射配置
LIO_TOPIC_CONFIGS = {
    'fast_lio': {
        'pointcloud_topic': '/cloud_registered_body',
        'odom_topic': '/Odometry',
        'octomap_topic': '/cloud_registered_body',
        'target_frame': 'base_footprint'
    },
    'lio_sam': {
        'pointcloud_topic': '/lio_sam/mapping/cloud_registered_raw',
        'odom_topic': '/lio_sam/mapping/odometry',
        'octomap_topic': '/lio_sam/mapping/cloud_registered',
        'target_frame': 'base_link'
    },
    'point_lio': {
        'pointcloud_topic': '/cloud_registered_body',
        'odom_topic': '/Odometry',
        'octomap_topic': '/cloud_registered_body',
        'target_frame': 'base_footprint'
    },
    'super_lio': {
        'pointcloud_topic': '/lio/body/cloud',
        'odom_topic': '/lio/odom',
        'octomap_topic': '/lio/body/cloud',
        'target_frame': 'base_footprint'
        # 'target_frame': 'base_link'
    }
}


def generate_launch_description():
    ns = LaunchConfiguration('ns', default='/')   # 默认 robot1
    # 定义启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)
    map_file = LaunchConfiguration('map_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    
    # 定义launch目录路径
    bringup_dir = get_package_share_directory('nav2_dog_slam')
    launch_dir = os.path.join(bringup_dir, 'launch')

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=NAV2_DEFAULT_MAP_FILE,
        description='Full path to map file to load')
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=NAV2_DEFAULT_PARAMS_FILE,
        description='Full path to the Nav2 parameters file')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='amcl',
        description='Localization backend to use: amcl or slam_toolbox'
    )
    
    # 根据SLAM_ALGORITHM参数选择启动不同的SLAM算法
    # 获取当前选择的LIO算法的话题配置
    lio_config = LIO_TOPIC_CONFIGS.get(SLAM_ALGORITHM, LIO_TOPIC_CONFIGS['fast_lio'])
    
    # FAST-LIO
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'fast_lio'"]))
    )
    
    # Point-LIO
    try:
        point_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('point_lio'), 'launch', 'mapping_mid360.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'point_lio'"]))
        )
    except Exception as e:
        print(f"Point-LIO package not found: {e}")
        # 创建一个空的动作作为占位符
        from launch.actions import LogInfo
        point_lio_launch = LogInfo(msg="Point-LIO package not found, skipping...")
    
    
    # LIO-SAM
    try:
        lio_sam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('lio_sam'), 'launch', 'lio_sam.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'lio_sam'"]))
        )
    except Exception as e:
        print(f"LIO-SAM package not found: {e}")
        # 创建一个空的动作作为占位符
        from launch.actions import LogInfo
        lio_sam_launch = LogInfo(msg="LIO-SAM package not found, skipping...")

    # Super-LIO
    try:
        super_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('super_lio'), 'launch', 'Livox_mid360.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'super_lio'"]))
        )
    except Exception as e:
        print(f"Super-LIO package not found: {e}")
        # 创建一个空的动作作为占位符
        from launch.actions import LogInfo
        super_lio_launch = LogInfo(msg="Super-LIO package not found, skipping...")
    













    # ============================================
    # 第一步：定义所有可能的节点
    # ============================================
    
    # 统一的节点配置
    unified_nodes = []
    nav2_actions = []
    web_actions = []
    
    # 1. 基础节点（所有模式都需要）
    # 注意：dynamic_base_footprint发布功能已迁移到各LIO算法的C++部分，此处不再需要
    
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
    
    # rosbridge_websocket节点
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'port': 9090},
            {'default_call_service_timeout': 5.0},
            {'call_services_in_new_thread': True},
            {'send_action_goals_in_new_thread': True}
        ]
    )
    
    # web控制脚本
    web_script_process = ExecuteProcess(
        cmd=['bash', NAV2_DEFAULT_WEB_SCRIPT_PATH],
        output='screen',
        shell=False
    )
    
    # 2. 建图工具节点
    # 合并后的slam_toolbox节点（支持建图和导航两种模式）
    slam_toolbox_params = LaunchConfiguration('slam_toolbox_params')
    declare_slam_toolbox_params_cmd = DeclareLaunchArgument(
        'slam_toolbox_params',
        default_value=NAV2_DEFAULT_PARAMS_FILE,
        description='Full path to slam_toolbox parameters file')
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_node',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {
                'use_sim_time': use_sim_time,
                'map_update_interval': 1.0,
                'publish_occupancy_map': 'True',
                'use_map_saver': True
            }
        ],
        prefix=['taskset -c 5,6'],
        remappings=[
            ('/scan', '/scan'), 
            ('/odom', lio_config['odom_topic']),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/initialpose', '/initialpose')
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # octomap_server节点
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'sensor_model/max_range': 100.0,
            'sensor_model/min_range': 0.4,
            'sensor_model/insert_free_space': 'True',
            'resolution': 0.05,
            'occupancy_min_z': -0.1,
            'occupancy_max_z': 1.0,
            'publish_2d_map': 'True',
            'use_sim_time': use_sim_time,
        }],
        prefix=['taskset -c 4,5'],
        remappings=[
            ('/cloud_in', lio_config['octomap_topic'])
        ]
    )
    
    # 3. 导航相关节点
    # map_server节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}
        ],
        prefix=['taskset -c 0,1,2,3'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # amcl节点
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        prefix=['taskset -c 5,6']
    )
    
    # 生命周期管理器节点
    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
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
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # 4. 导航栈节点
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'container_name': 'nav2_container',
            'log_level': 'info'
        }.items()
    )

    sc_pgo_node = Node(
        package="sc_pgo_ros2",
        executable="alaserPGO",
        name="alaserPGO",
        output="screen",
        parameters=[
            {"scan_line": 4},
            {"minimum_range": 0.3},
            {"mapping_line_resolution": 0.4},
            {"mapping_plane_resolution": 0.8},
            {"mapviz_filter_size": 0.05},
            {"keyframe_meter_gap": 1.0},
            {"sc_dist_thres": 0.3},
            {"sc_max_radius": 290.0},
            {"save_directory": "/home/ztl/save_data/"},  # 修改为实际保存路径
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/aft_mapped_to_init", "/Odometry"),
            # ("/aft_mapped_to_init", "/aft_mapped_to_init"),
            ("/velodyne_cloud_registered_local", "/cloud_registered_body"),
            ("/cloud_for_scancontext", "/cloud_registered_body"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        prefix=['taskset -c 6'],   # 绑定 CPU 6
    )
    









    # ============================================
    # 第二步：根据配置条件决定启动哪些节点
    # ============================================
    
    # 1. 基础节点（所有模式都需要）
    unified_nodes.append(pointcloud_to_laserscan_node)
    
    # 2. Web相关节点（所有模式都需要）
    web_actions.append(web_script_process)
    web_actions.append(rosbridge_websocket)
    
    # 3. 建图模式配置
    if MANUAL_BUILD_MAP:
        if BUILD_TOOL == 'slam_toolbox':
            # 建图模式 + slam_toolbox
            unified_nodes.append(declare_slam_toolbox_params_cmd)
            unified_nodes.append(
                TimerAction(
                    period=5.0,
                    actions=[slam_toolbox_node]
                )
            )
        
        else:
            # 建图模式：添加octomap server
            unified_nodes.append(
                TimerAction(
                    period=15.0,
                    actions=[octomap_server_node]
                )
            )

    if MANUAL_BUILD_MAP or AUTO_BUILD_MAP:
        # 建图模式 + SC-PGO
        unified_nodes.append(
            TimerAction(
                period=10.0,  # 延迟10秒启动SC-PGO，确保LIO算法已初始化
                actions=[sc_pgo_node]
            )
        )
    

        
    if not MANUAL_BUILD_MAP and not AUTO_BUILD_MAP:
        nav2_actions.append(
            TimerAction(
                period=1.0,
                actions=[map_server_node]
            )
        )
        nav2_actions.append(
            TimerAction(
                period=1.5,
                actions=[lifecycle_manager_map_server]
            )
        )

    # 4. 导航模式配置
    if BUILD_TOOL != 'slam_toolbox' :
        # 根据localization参数选择AMCL或SLAM Toolbox
        nav2_actions.append(
            TimerAction(
                period=2.0,
                actions=[amcl_node],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' != 'slam_toolbox'"]))
            )
        )
        nav2_actions.append(
            TimerAction(
                period=2.0,
                actions=[lifecycle_manager_amcl],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' != 'slam_toolbox'"]))
            )
        )
        
        # SLAM Toolbox导航模式
        nav2_actions.append(
            TimerAction(
                period=2.0,
                actions=[slam_toolbox_node],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'slam_toolbox'"]))
            )
        )
        
        
    # 导航栈
    nav2_actions.append(
        TimerAction(
            period=3.0,
            actions=[navigation_include]
        )
    )
    
    # 创建延迟启动动作
    delayed_web_launch = TimerAction(
        period=5.0,  # 延迟5秒启动web控制脚本
        actions=web_actions
    )
    
    # if nav2_actions:
    delayed_nav2_launch = TimerAction(
        period=5.0,  # 延迟5秒启动Nav2，确保SLAM算法已初始化
        actions=nav2_actions
    )
    
    # 创建并返回完整的launch description
    # 注意: 启动参数声明必须在使用它们的操作之前
    launch_actions = [
        PushRosNamespace(ns),
        # 1. 声明所有启动参数
        declare_map_file_cmd,
        declare_localization_cmd,
        declare_nav2_params_file_cmd,
        # 2. 启动主要组件（根据SLAM_ALGORITHM参数选择）
        fast_lio_launch,
        point_lio_launch,
        lio_sam_launch,
        super_lio_launch,
        # 3. 添加统一的节点配置
        *unified_nodes
    ]
    
    # 3. 添加web_actions（固定都要启动）
    launch_actions.append(delayed_web_launch)
    
    # # 4. 根据模式添加nav2_actions
    # if MANUAL_BUILD_MAP:
    #     # 建图模式：不需要添加Nav2
    #     pass
    # elif 'delayed_nav2_launch' in locals():
    #     # 导航模式：添加nav2
    
    launch_actions.append(delayed_nav2_launch)
    
    # 5. 如果AUTO_BUILD_MAP为True，延迟启动explore_lite
    if AUTO_BUILD_MAP and not MANUAL_BUILD_MAP:
        explore_lite_package_dir = get_package_share_directory('explore_lite')
        explore_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                explore_lite_package_dir, 'launch', 'explore.launch.py')])
        )
        delayed_explore_launch = TimerAction(
            period=60.0,  # 延迟60秒启动explore_lite
            actions=[explore_launch]
        )
        launch_actions.append(delayed_explore_launch)
    
    return LaunchDescription(launch_actions)