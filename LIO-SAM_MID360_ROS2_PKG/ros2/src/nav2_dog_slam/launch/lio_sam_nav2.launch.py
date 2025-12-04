from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

# 尝试从标准Python包导入配置参数
import sys, os
# 获取包的共享目录
lio_sam_package_dir = get_package_share_directory('lio_sam')
sys.path.insert(0, lio_sam_package_dir + '/launch')

# 导入全局配置
from ament_index_python.packages import get_package_share_directory
import sys, os

# 添加global_config包的路径到Python路径
try:
    global_config_path = get_package_share_directory('global_config')
    if global_config_path not in sys.path:
        sys.path.insert(0, global_config_path)
    from global_config import (
        BUILD_MAP, BUILD_TOOL, RECORD_ONLY, NAVIGATION_MODE, 
        ONLINE_LIDAR as ONLINE_LIDAR, 
        LIO_SAM_BASE_CODE_PATH as BASE_CODE_PATH, 
        DEFAULT_USE_SIM_TIME as DEFAULT_USE_SIM_TIME,
        DEFAULT_USE_SIM_TIME_STRING as DEFAULT_USE_SIM_TIME_STRING, 
        DEFAULT_BAG_PATH as DEFAULT_BAG_PATH,
        DEFAULT_RELIABILITY_OVERRIDE as DEFAULT_RELIABILITY_OVERRIDE, 
        LIO_SAM_DEFAULT_LOAM_SAVE_DIR as DEFAULT_LOAM_SAVE_DIR,
        NAV2_BASE_CODE_PATH, NAV2_DEFAULT_MAP_FILE, NAV2_DEFAULT_WEB_SCRIPT_PATH,
        NAV2_DEFAULT_BT_XML_PATH, 
        DEFAULT_USE_SIM_TIME_STRING, MAP_FRAME, ODOM_FRAME, 
        BASE_LINK_FRAME, LIVOX_FRAME
    )
except Exception as e:
    print(f"导入global_config失败: {e}")
    # 如果导入失败，使用默认值
    BUILD_MAP = False
    BUILD_TOOL = 'octomap_server'
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
    NAV2_DEFAULT_MAP_FILE = "/home/ztl/slam_data/grid_map/map.yaml"
    NAV2_DEFAULT_WEB_SCRIPT_PATH = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh'
    NAV2_DEFAULT_BT_XML_PATH = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
    MAP_FRAME = 'map'
    ODOM_FRAME = 'odom'
    BASE_LINK_FRAME = 'base_link'
    LIVOX_FRAME = 'livox_frame'

# 获取当前launch文件所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))
    


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
        default_value=os.path.join(current_dir, '../config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='amcl',
        description='Localization backend to use: amcl or slam_toolbox'
    )
    
    # 集成模式：启动LIO-SAM
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            lio_sam_package_dir, 'launch', 'lio_sam.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    nav2_and_web_actions = []
    if not BUILD_MAP:
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
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'amcl'"]))
        )

        nav2_slam_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                current_dir, 'nav2_slam_toolbox.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
                'slam_toolbox_params': os.path.join(package_dir, 'config', 'nav2_params.yaml')
            }.items(),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'slam_toolbox'"]))
        )

        # 延迟启动Nav2的launch文件（两个 include 都会被放入延迟动作中，条件决定实际生效的那个）
        # Append both include actions; each include has its own condition so only the
        # matching one (amcl or slam_toolbox) will actually be executed at runtime.
        nav2_and_web_actions.append(nav2_amcl_include)
        nav2_and_web_actions.append(nav2_slam_include)
        
        # # 延迟调用AMCL的全局定位服务
        # nav2_and_web_actions.append(TimerAction(
        #     period=5.0,  # Nav2启动后再延迟5秒调用服务
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['ros2', 'service', 'call', '/reinitialize_global_localization', 'std_srvs/srv/Empty'],
        #             output='screen',
        #             shell=False
        #         )
        #     ]
        # ))
        
        # 调用web控制脚本
        nav2_and_web_actions.append(ExecuteProcess(
            cmd=['bash', NAV2_DEFAULT_WEB_SCRIPT_PATH],
            output='screen',
            shell=False
        ))
    
    # 
    # 根据模式创建适当的延迟启动动作
    if nav2_and_web_actions:
        delayed_nav2_launch = TimerAction(
            period=5.0,  # 延迟5秒启动Nav2，确保LIO-SAM已初始化
            actions=nav2_and_web_actions
        )
    
    # 创建并返回完整的launch description
    # 注意: 启动参数声明必须在使用它们的操作之前
    launch_actions = [
        PushRosNamespace(ns),
        # 1. 声明所有启动参数
        declare_map_file_cmd,
        declare_localization_cmd,
        declare_nav2_params_file_cmd,
        # 2. 启动主要组件
        lio_sam_launch
    ]
    
    # 3. 根据模式添加相应的节点
    if  BUILD_MAP:
        # 建图模式：添加octomap server
        pass
    elif 'delayed_nav2_launch' in locals():
        # 导航模式：添加nav2和web
        launch_actions.append(delayed_nav2_launch)
    
    return LaunchDescription(launch_actions)