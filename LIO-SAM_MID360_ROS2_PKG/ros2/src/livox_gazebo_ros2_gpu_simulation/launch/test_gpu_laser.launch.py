#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 设置Gazebo环境变量
    gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        '/usr/share/gazebo-11'
    )
    
    gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib'
    )
    
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        '/usr/share/gazebo-11/models'
    )
    
    ogre_resource_path = SetEnvironmentVariable(
        'OGRE_RESOURCE_PATH',
        '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0'
    )
    
    # 声明启动参数
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='',
        description='Path to the world file'
    )
    
    # 启动Gazebo（如果world_file为空，只启动空世界）
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', LaunchConfiguration('world_file')],
        output='screen',
        shell=False
    )
    
    # 启动激光雷达数据监听节点
    laser_listener = Node(
        package='livox_gazebo_ros2_gpu_simulation',
        executable='laser_listener',
        name='laser_listener',
        output='screen'
    )
    
    return LaunchDescription([
        # 设置环境变量
        gazebo_resource_path,
        gazebo_plugin_path,
        gazebo_model_path,
        ogre_resource_path,
        # 启动参数和进程
        world_file_arg,
        gazebo_process,
        laser_listener,
    ])
