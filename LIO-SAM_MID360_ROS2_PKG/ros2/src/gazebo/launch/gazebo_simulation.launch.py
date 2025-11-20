#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directory
    pkg_gazebo = get_package_share_directory('gazebo')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='lio_sam_world.world')
    
    # Robot description
    robot_description_path = os.path.join(pkg_gazebo, 'urdf', 'robot.urdf')
    
    # Read robot description
    with open(robot_description_path, 'r') as file:
        robot_description = file.read()
    
    # Gazebo launch
    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'gazebo_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.3']
    )
    
    # RViz2
    rviz_config_path = os.path.join(pkg_gazebo, 'config', 'gazebo_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='lio_sam_world.world',
            description='Gazebo world file'
        ),
        
        # Include Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'world': PathJoinSubstitution([pkg_lio_sam, 'worlds', world_file])}.items()
        ),
        
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        rviz_node
    ])