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
    'dlio': {
        'pointcloud_topic': '/dlio/odom_node/pointcloud/deskewed',
        'odom_topic': '/Odometry',
        'octomap_topic': '/dlio/odom_node/pointcloud/deskewed',
        'target_frame': 'base_link'
    },
    'faster_lio': {
        'pointcloud_topic': '/cloud_registered_body',
        'odom_topic': '/Odometry',
        'octomap_topic': '/cloud_registered_body',
        'target_frame': 'base_link'
    },
    'point_lio': {
        'pointcloud_topic': '/cloud_registered_body',
        'odom_topic': '/Odometry',
        'octomap_topic': '/cloud_registered_body',
        'target_frame': 'base_footprint'
    }
}


def generate_launch_description():
    ns = LaunchConfiguration('ns', default='/')   # 默认 robot1
    # 定义启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)
    map_file = LaunchConfiguration('map_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

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
    
    # Faster-LIO
    try:
        faster_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('faster_lio'), 'launch_ros2', 'fasterlio_launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'faster_lio'"]))
        )
    except Exception as e:
        print(f"Faster-LIO package not found: {e}")
        # 创建一个空的动作作为占位符
        from launch.actions import LogInfo
        faster_lio_launch = LogInfo(msg="Faster-LIO package not found, skipping...")
    
    # DLIO
    try:
        dlio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('direct_lidar_inertial_odometry'), 'launch', 'dlio.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(PythonExpression(["'", SLAM_ALGORITHM, "' == 'dlio'"]))
        )
    except Exception as e:
        print(f"DLIO package not found: {e}")
        # 创建一个空的动作作为占位符
        from launch.actions import LogInfo
        dlio_launch = LogInfo(msg="DLIO package not found, skipping...")
    
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
    
    # 统一的节点配置
    unified_nodes = []
    
    # 动态base_footprint发布器节点（使用ExecuteProcess方式）
    dynamic_base_footprint_process = ExecuteProcess(
        cmd=['python3', get_package_share_directory('nav2_dog_slam') + '/../../lib/nav2_dog_slam/dynamic_base_footprint.py',
             '--base_link_frame', BASE_LINK_FRAME,
             '--base_footprint_frame', 'base_footprint',
             '--odom_frame', ODOM_FRAME,
             '--use_sim_time', use_sim_time],
        name='dynamic_base_footprint',
        output='screen',
        shell=False
    )
    unified_nodes.append(dynamic_base_footprint_process)
    
    # PointCloud to LaserScan 节点（所有LIO算法都需要）
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
            'max_height': 1.5,
            'angle_min': -3.1,
            'angle_max': 3.1,
            'angle_increment': 0.00869347338,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 100.0,
            'use_inf': False,
            'inf_epsilon': 100.0,
            'use_sim_time': use_sim_time,
            'target_frame': lio_config['target_frame'],
            'concurrency_level': 1,
        }],
        output='screen',
        prefix=['taskset -c 5'],
    )
    unified_nodes.append(pointcloud_to_laserscan_node)
    
    # slam_toolbox_node（仅在BUILD_TOOL为slam_toolbox时启动）
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
                'publish_occupancy_map': True,
                'use_map_saver': True
            }
        ],
        prefix=['taskset -c 5,6'],
        remappings=[('/scan', '/scan'), ('/odom', lio_config['odom_topic'])]
    )
    
    # octomap_server_node（仅在BUILD_TOOL为octomap_server时启动）
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'sensor_model/max_range': 100.0,
            'sensor_model/min_range': 0.4,
            'sensor_model/insert_free_space': True,
            'resolution': 0.05,
            'occupancy_min_z': -0.1,
            'occupancy_max_z': 1.0,
            'publish_2d_map': True,
            'use_sim_time': use_sim_time,
        }],
        prefix=['taskset -c 4,5'],
        remappings=[
            ('/cloud_in', lio_config['octomap_topic'])
        ]
    )
    
    nav2_actions = []
    web_actions = []
    
    # web_actions固定都要启动
    web_actions.append(ExecuteProcess(
        cmd=['bash', NAV2_DEFAULT_WEB_SCRIPT_PATH],
        output='screen',
        shell=False
    ))
    
    # if not MANUAL_BUILD_MAP:
    if BUILD_TOOL != 'slam_toolbox':
        # 根据 localization 参数选择包含哪一个 nav2 启动文件（amcl 或 slam_toolbox）
        # For AMCL we use a dedicated local launch that starts map_server + amcl
        # and forwards the main params file. This avoids the need for a second
        # YAML file and lets the nodes read their sections from nav2_params.yaml.
        nav2_amcl_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                current_dir, 'nav2_amcl.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': nav2_params_file,
                'autostart': 'True',
                'use_composition': 'True',
                'use_respawn': 'False',
                'log_level': 'info'
            }.items(),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' != 'slam_toolbox'"]))
        )

        nav2_slam_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                current_dir, 'nav2_slam_toolbox.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': NAV2_DEFAULT_PARAMS_FILE,
                'slam_toolbox_params': NAV2_DEFAULT_PARAMS_FILE
            }.items(),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'slam_toolbox'"]))
        )

        # 将nav2启动文件添加到nav2_actions中
        nav2_actions.append(nav2_amcl_include)
        nav2_actions.append(nav2_slam_include)
    
    # 根据BUILD_TOOL参数添加相应的节点
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
        faster_lio_launch,
        dlio_launch,
        lio_sam_launch,
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