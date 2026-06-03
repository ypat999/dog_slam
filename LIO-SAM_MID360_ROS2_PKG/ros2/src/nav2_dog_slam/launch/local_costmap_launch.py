# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, PushRosNamespace
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = 'rkbot'


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')


    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')


    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params_zg_3d.yaml'

    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    map_frame = 'rkbot/map'
    odom_frame = 'rkbot/odom'
    base_frame = 'rkbot/base_footprint'
    scan_topic = '/rkbot/scan'
    pointcloud_topic = '/rkbot/front_lidar/cloud_world'

    map_topic = '/rkbot/map'

    lifecycle_nodes = ['controller_server']

    remappings = [('tf', 'tf'),
                  ('tf_static', 'tf_static'),
                  ('initialpose', 'initialpose'),
                  ('goal_pose', 'goal_pose')]

    param_substitutions_local = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'global_frame': odom_frame,
        'robot_base_frame': base_frame,
        'global_frame_id': map_frame,
        'odom_frame_id': odom_frame,
        'base_frame_id': base_frame,
        'map_frame': map_frame,
        'odom_frame': odom_frame,
        'base_frame': base_frame,
        'map_topic': map_topic,
        # 'topic': scan_topic
    }

    configured_params_local = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions_local,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    

    load_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params_local],
                arguments=['--ros-args', '--log-level', log_level],
                prefix=['taskset -c 4,5,6,7'],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_local_costmap',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                prefix=['taskset -c 0,1,2,3'],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    
    ld.add_action(load_nodes)

    return ld
