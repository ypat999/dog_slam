import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    IncludeLaunchDescription,
    LogInfo,
    Shutdown
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robot_nav2')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='none',
        description='Reference path to the map yaml file',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='NO_SPECIFIED',
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_gazebo_params_file_cmd = DeclareLaunchArgument(
        'gazebo_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'GAZEBO', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_xg_params_file_cmd = DeclareLaunchArgument(
        'xg_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'XG', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_zg_params_file_cmd = DeclareLaunchArgument(
        'zg_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'ZG', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_platform_cmd = DeclareLaunchArgument(
        'platform',
        default_value='REAL',
        description='platform of nav running on'
    )

    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type',
        default_value='xg',
        description='Robot type for loading geometry configuration (xg, zg, default)'
    )

    declare_mc_controller_type_cmd = DeclareLaunchArgument(
        'mc_controller_type',
        default_value='RL_TRACK_VELOCITY',
        description='mc controller type used in mc'
    )

    set_map = SetLaunchConfiguration(
        'map',
        PythonExpression([
            "'",
            "",
            "' if '",
            LaunchConfiguration('map'),
            "' == 'none' else '",
            LaunchConfiguration('map'),
            "'"
        ])
    )

    set_use_sim_time = SetLaunchConfiguration(
        'use_sim_time',
        PythonExpression([
            "'",
            'true',
            "' if '",
            LaunchConfiguration('platform'),
            "' == 'GAZEBO' else '",
            LaunchConfiguration('use_sim_time'),
            "'"
        ])
    )

    set_params_file = SetLaunchConfiguration(
        'params_file',
        PythonExpression([
            "'",
            LaunchConfiguration('gazebo_params_file'),
            "' if '",
            LaunchConfiguration('platform'),
            "' == 'GAZEBO' else '",
            LaunchConfiguration('xg_params_file'),
            "' if ('",
            LaunchConfiguration('platform'),
            "' == 'REAL' and '",
            LaunchConfiguration('robot_type'),
            "' == 'xg') else '",
            LaunchConfiguration('zg_params_file'),
            "' if ('",
            LaunchConfiguration('platform'),
            "' == 'REAL' and '",
            LaunchConfiguration('robot_type'),
            "' == 'zg') else 'NO_SPECIFIED'"
        ])
    )

    platform_log = LogInfo(
        msg=['platform: ', LaunchConfiguration('platform')]
    )

    robot_type_log = LogInfo(
        msg=['robot_type: ', LaunchConfiguration('robot_type')]
    )

    params_file_log = LogInfo(
        msg=['params_file: ', LaunchConfiguration('params_file')]
    )

    map_log = LogInfo(
        msg=['map: ', LaunchConfiguration('map')]
    )

    mc_controller_type_log = LogInfo(
        msg=['mc_controller_type: ', LaunchConfiguration('mc_controller_type')]
    )

    use_sim_time_log = LogInfo(
        msg=['use_sim_time: ', LaunchConfiguration('use_sim_time')]
    )

    platform_warn_log = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('platform'), "' == 'NO_SPECIFIED'"])),
        msg="\033[1;31mWarning: You have not specified the 'platform'. [platform:=GAZEBO || platform:=REAL] Early Terminated\033[0m"
    )

    params_file_warn_log = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('params_file'), "' == 'NO_SPECIFIED'"])),
        msg="\033[1;31mWarning: You have not specified the 'params_file'. Early Terminated\033[0m"
    )

    mc_controller_type_warn_log = LogInfo(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mc_controller_type'), "' == 'NO_SPECIFIED'"])),
        msg="\033[1;31mWarning: You have not specified the 'mc_controller_type'. [mc_controller_type:=RL_TRACK_VELOCITY || mc_controller_type:=RL_TRACK_PATH] Early Terminated\033[0m"
    )

    shutdown_if_no_platform_type = Shutdown(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('platform'), "' == 'NO_SPECIFIED'"]))
    )

    shutdown_if_no_params_file = Shutdown(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('params_file'), "' == 'NO_SPECIFIED'"]))
    )

    shutdown_if_no_mc_controller_type = Shutdown(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mc_controller_type'), "' == 'NO_SPECIFIED'"]))
    )

    # odom_communication_node = Node(
    #     package='robot_nav2',
    #     executable='odom_communication_node',
    #     output='screen'
    # )

    # custom_odom_baselink_node = Node(
    #     package='robot_nav2',
    #     executable='custom_odom_baselink_node',
    #     output='screen'
    # )

    # odom_tf_publisher_node = Node(
    #     package='robot_nav2',
    #     executable='odom_to_tf_broadcaster',
    #     output='screen'
    # )

    # cmd_cel_lcm_publisher_node = Node(
    #     package='robot_nav2',
    #     executable='vel_cmd_lcm_pub',
    #     output='screen'
    # )

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    robot_type = LaunchConfiguration('robot_type')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'robot_type': robot_type,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_gazebo_params_file_cmd)
    ld.add_action(declare_xg_params_file_cmd)
    ld.add_action(declare_zg_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_platform_cmd)
    ld.add_action(declare_robot_type_cmd)
    ld.add_action(declare_mc_controller_type_cmd)

    ld.add_action(set_map)
    ld.add_action(set_use_sim_time)
    ld.add_action(set_params_file)

    ld.add_action(platform_log)
    ld.add_action(robot_type_log)
    ld.add_action(params_file_log)
    ld.add_action(map_log)
    ld.add_action(mc_controller_type_log)
    ld.add_action(use_sim_time_log)
    ld.add_action(platform_warn_log)
    ld.add_action(params_file_warn_log)
    ld.add_action(mc_controller_type_warn_log)
    ld.add_action(shutdown_if_no_platform_type)
    ld.add_action(shutdown_if_no_params_file)
    ld.add_action(shutdown_if_no_mc_controller_type)

    # ld.add_action(odom_communication_node)
    # ld.add_action(custom_odom_baselink_node)
    # ld.add_action(odom_tf_publisher_node)
    # ld.add_action(cmd_cel_lcm_publisher_node)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd)

    return ld
