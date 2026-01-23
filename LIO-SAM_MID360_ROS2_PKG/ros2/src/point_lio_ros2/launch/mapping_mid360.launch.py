from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import sys, os

def generate_launch_description():
    try:
        # 方法1：通过ROS2包路径导入
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            ONLINE_LIDAR, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
            DEFAULT_USE_SIM_TIME, MANUAL_BUILD_MAP, BUILD_TOOL, RECORD_ONLY,
            NAV2_DEFAULT_PARAMS_FILE
        )
    except ImportError as e:
        print(f"方法2导入global_config失败: {e}")
        # 如果导入失败，使用默认值
        ONLINE_LIDAR = True
        DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
        DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
        DEFAULT_USE_SIM_TIME = False
        MANUAL_BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        RECORD_ONLY = False
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'

    # 导入全局配置
    from global_config import LIVOX_MID360_CONFIG_NO_TILT
    
    livox_config_path = LIVOX_MID360_CONFIG_NO_TILT
    lidar_mode = "ONLINE"
    if not ONLINE_LIDAR:
        lidar_mode = "OFFLINE"

    use_sim_time = LaunchConfiguration('use_sim_time', default=str(DEFAULT_USE_SIM_TIME))

    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'mid360.yaml'
        ]),
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,  # Options: 1, 3
            'space_down_sample': True,
            'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
            'use_sim_time': DEFAULT_USE_SIM_TIME,
        }
    ]


    # 在线模式：Livox雷达驱动
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 40.0},
            {"output_data_type": 0},
            {"frame_id": 'livox_frame'},
            {"user_config_path": livox_config_path},
            {"cmdline_input_bd_code": 'livox0000000001'},
        ],
        prefix=['taskset -c 4'],   # 绑定 CPU 4
        condition=IfCondition(PythonExpression("'" + lidar_mode + "' == 'ONLINE'")),
    )

  



    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )

    # # odom -> base_link (里程计到机器人基坐标系的静态变换)
    # static_transform_odom_to_base_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_odom_to_base_link',
    #     parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'],
    #     output='screen'
    # )

    # base_link_to_livox_frame_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_livox_frame_tf',
    #     parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
    #     arguments=['0.1', '0', '0.1', '0', '0.5235987756', '0', 'base_link', 'livox_frame'],
    #     output='screen'
    # )

    # odom -> base_link (里程计到机器人基坐标系的静态变换)
    static_transform_odom_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_livox_frame',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.5', '0.0', '0.0', '0.0', 'odom', 'base_link'],
        output='screen'
    )

    base_link_to_livox_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.1', '0', '0.1', '0', '0.5235987756', '0', 'base_link', 'livox_frame'],
        output='screen'
    )





    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        prefix=['taskset -c 7'],   # 绑定 CPU 7
        # prefix='gdb -ex run --args'
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        # rviz_arg,
        livox_driver_node,
        static_transform_map_to_odom,
        # static_transform_odom_to_base_link,
        # base_link_to_livox_frame_tf,
        static_transform_odom_to_livox_frame,
        base_link_to_livox_frame_tf,
        laser_mapping_node,
        # GroupAction(
        #     actions=[rviz_node],
        #     condition=IfCondition(LaunchConfiguration('rviz'))
        # ),
    ])

    return ld
