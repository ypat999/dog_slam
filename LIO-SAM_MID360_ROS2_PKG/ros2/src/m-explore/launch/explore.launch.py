import sys, os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 导入全局配置
try:
    global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
    sys.path.insert(0, global_config_path)
    from global_config import *
except ImportError:
    print("Warning: Failed to import global_config, using default values")
    DEFAULT_USE_SIM_TIME = False


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params_costmap.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true" if DEFAULT_USE_SIM_TIME else "false", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]


    nav2_dir = get_package_share_directory('nav2_dog_slam')
    slam_toolbox_params = LaunchConfiguration('slam_toolbox_params')
    declare_slam_toolbox_params_cmd = DeclareLaunchArgument(
        'slam_toolbox_params',
        default_value=os.path.join(nav2_dir, 'config', 'nav2_params.yaml'),
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
            }
        ],
        remappings=[
            ('/scan', '/scan'), 
            # ('/odom', '/lio_sam/mapping/odometry'),
            ('/odom', '/Odometry'),  # FAST-LIO 的 odometry 话题
            ('/initialpose', '/initialpose')  # Enable initial pose setting
        ],
        respawn=True,  # 启用自动重启，防止崩溃后系统停止运行
        respawn_delay=1.0  # 重启延迟10秒，给系统足够时间稳定
    )



    node = Node(
        package="explore_lite",
        name="explore_node",
        namespace=namespace,
        executable="explore",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
    )
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(node)

    #使用amcl与slamtoolbox混合
    # ld.add_action(declare_slam_toolbox_params_cmd)
    # ld.add_action(slam_toolbox_node)

    return ld
