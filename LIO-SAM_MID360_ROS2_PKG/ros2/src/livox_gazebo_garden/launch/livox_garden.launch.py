#!/usr/bin/env python3
"""
Livox LiDAR Gazebo Garden 启动文件
用于启动迁移后的 Livox 仿真环境 (Gazebo Garden + ROS 2)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('livox_gazebo_garden')

    env_vars = [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
            f'{os.path.expanduser("~/.gazebo/models")}:'
            f'{pkg_dir}/worlds:'
            f'/usr/share/gz/gz-sim'),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH',
            f'{pkg_dir}:'
            f'/usr/share/gz/gz-sim'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
            f'{pkg_dir}/worlds:'
            f'{pkg_dir}:'
            f'/usr/share/gz/gz-sim'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
            f'{pkg_dir}/lib:'
            '/opt/ros/humble/lib:'
            '${GZ_SIM_SYSTEM_PLUGIN_PATH}'),
    ]

    # world_file = 'test_zone.world'
    world_file = 'ego.sdf'

    gazebo_process = ExecuteProcess(
        cmd=['taskset', '-c', '0,1,2,3', 'gz', 'sim', '-r', os.path.join(pkg_dir, 'worlds', world_file)],
        output='screen',
    )

    sdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot', 'my_robot.sdf')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot', 'my_robot.urdf')
    
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'publish_frequency': 10.0,
        }],
        output='screen',
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-file', sdf_path,
            '-name', 'mid360_robot',
            '-world', world_file.split('.')[0],
            '-x', '0.0', '-y', '0.0', '-z', '0.5',
            '-timeout', '60.0',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    bridge_config = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
        prefix=['taskset -c 0,1,2,3'],
    )

    # joint_state_publisher 已禁用以节省CPU
    # 如需关节状态可视化，取消注释以下代码
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': True, 'rate': 10.0}],
    #     output='screen',
    # )

    static_transform_livox_frame_to_mid360_robot_livox_frame_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_livox_frame_to_mid360_robot_livox_frame_lidar',
        parameters=[{'use_sim_time': True}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'livox_frame', 'mid360_robot/livox_frame/lidar'],
        output='screen'
    )

    rviz_config = os.path.join(pkg_dir, 'rviz', 'my_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        prefix=['taskset -c 0,1,2,3'],
    )


    return LaunchDescription([
        *env_vars,
        gazebo_process,
        robot_state_publisher,
        TimerAction(period=15.0, actions=[spawn_entity]),
        static_transform_livox_frame_to_mid360_robot_livox_frame_lidar,
        TimerAction(period=20.0, actions=[
            bridge_node,
            # joint_state_publisher,  # 已禁用
            rviz_node,
        ]),
    ])
