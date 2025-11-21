# Nav2 Dog SLAM 全局配置参数
# 此文件作为标准Python模块，可以被其他模块导入使用
import os
import platform

# 获取当前主机名
current_machine = platform.node()
print(f"当前运行主机: {current_machine}")

# 导航模式配置
NAVIGATION_MODE = os.environ.get('NAVIGATION_MODE', 'standalone').lower()  # standalone: 独立模式, integrated: 集成模式

# 定义主机特定的配置字典
config_by_machine = {
    'RK3588': {
        # RK3588主机配置
        'BASE_CODE_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
        'DEFAULT_MAP_FILE': "/home/ztl/slam_data/grid_map/map.yaml",
        'DEFAULT_WEB_SCRIPT_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web_150.sh',
        'DEFAULT_BT_XML_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/bt_straight_then_rotate.xml',
        'DEFAULT_USE_SIM_TIME': False,
        'DEFAULT_USE_SIM_TIME_STRING': 'false'
    },
    'jqr001': {
        # jqr001主机配置
        'BASE_CODE_PATH': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
        'DEFAULT_MAP_FILE': '/home/ywj/projects/map_grid/map.yaml',
        'DEFAULT_WEB_SCRIPT_PATH': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web_137.sh',
        'DEFAULT_BT_XML_PATH': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/bt_straight_then_rotate.xml',
        'DEFAULT_USE_SIM_TIME': True,
        'DEFAULT_USE_SIM_TIME_STRING': 'true'
    },
    'DESKTOP-4LS1SSN': {
        # DESKTOP-4LS1SSN主机配置
        'BASE_CODE_PATH': '/mnt/d/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
        'DEFAULT_MAP_FILE': "/mnt/d/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/map_sample/map.yaml",
        'DEFAULT_WEB_SCRIPT_PATH': '/mnt/d/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web_notebook.sh',
        'DEFAULT_BT_XML_PATH': '/mnt/d/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/bt_straight_then_rotate.xml',
        'DEFAULT_USE_SIM_TIME': True,
        'DEFAULT_USE_SIM_TIME_STRING': 'true'
    }
}

# 默认配置（当主机名不在配置字典中时使用）
default_config = {
    'BASE_CODE_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
    'DEFAULT_MAP_FILE': "/home/ztl/slam_data/grid_map/map.yaml",
    'DEFAULT_WEB_SCRIPT_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh',
    'DEFAULT_BT_XML_PATH': '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/bt_straight_then_rotate.xml',
    'DEFAULT_USE_SIM_TIME': True,
    'DEFAULT_USE_SIM_TIME_STRING': 'true'
}

# 根据当前主机名选择配置
selected_config = config_by_machine.get(current_machine, default_config)

# 将配置参数导出为模块变量
BASE_CODE_PATH = selected_config['BASE_CODE_PATH']
DEFAULT_MAP_FILE = selected_config['DEFAULT_MAP_FILE']
DEFAULT_WEB_SCRIPT_PATH = selected_config['DEFAULT_WEB_SCRIPT_PATH']
DEFAULT_BT_XML_PATH = selected_config['DEFAULT_BT_XML_PATH']
DEFAULT_USE_SIM_TIME = selected_config['DEFAULT_USE_SIM_TIME']
DEFAULT_USE_SIM_TIME_STRING = selected_config['DEFAULT_USE_SIM_TIME_STRING']

# 更新nav2_params.yaml中的配置
nav2_params_path = os.path.join(BASE_CODE_PATH, 'config/nav2_params.yaml')
if os.path.exists(nav2_params_path):
    with open(nav2_params_path, 'r') as file:
        lines = file.readlines()
    with open(nav2_params_path, 'w') as file:
        for line in lines:
            if '      use_sim_time' in line:
                file.write(f'      use_sim_time: {DEFAULT_USE_SIM_TIME_STRING}\n')
            elif '    use_sim_time:' in line:
                file.write(f'    use_sim_time: {DEFAULT_USE_SIM_TIME_STRING}\n')
            elif 'yaml_filename:' in line:
                file.write(f'    yaml_filename: {DEFAULT_MAP_FILE}\n')
            elif 'default_nav_to_pose_bt_xml:' in line:
                file.write(f'    default_nav_to_pose_bt_xml: "{DEFAULT_BT_XML_PATH}"\n')
            else:
                file.write(line)

# 坐标系名称配置
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_LINK_FRAME = 'base_link'

# 话题名称配置
ODOM_TOPIC = '/odometry/global'
SCAN_TOPIC = '/scan'
CMD_VEL_TOPIC = '/cmd_vel'

# 服务名称配置
INITIAL_POSE_SERVICE = '/initialpose'
GOAL_POSE_SERVICE = '/goal_pose'