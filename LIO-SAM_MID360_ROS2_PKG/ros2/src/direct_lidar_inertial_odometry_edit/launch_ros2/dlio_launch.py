import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'direct_lidar_inertial_odometry'
    package_dir = get_package_share_directory(package_name)
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable RViz visualization'
    )
    
    # Path to config files
    dlio_config = os.path.join(package_dir, 'cfg', 'dlio.yaml')
    params_config = os.path.join(package_dir, 'cfg', 'params.yaml')
    
    # DLIO Odometry Node
    dlio_odom_node = Node(
        package=package_name,
        executable='dlio_odom_node',
        name='dlio_odom',
        output='screen',
        parameters=[
            dlio_config,
            params_config,
            {
                'dlio.pointcloud.deskew': True,
                'dlio.pointcloud.voxelize': True,
                'dlio.imu.calibration': True
            }
        ],
        remappings=[
            ('~/pointcloud', '/livox/lidar'),
            ('~/imu', '/livox/imu'),
            ('~/odom', 'dlio/odom_node/odom'),
            ('~/pose', 'dlio/odom_node/pose'),
            ('~/path', 'dlio/odom_node/path'),
            ('~/kf_pose', 'dlio/odom_node/keyframes'),
            ('~/kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('~/deskewed', 'dlio/odom_node/pointcloud/deskewed')
        ]
    )
    
    # DLIO Mapping Node
    dlio_map_node = Node(
        package=package_name,
        executable='dlio_map_node',
        name='dlio_map',
        output='screen',
        parameters=[
            dlio_config,
            params_config
        ],
        remappings=[
            ('~/keyframes', 'dlio/odom_node/pointcloud/keyframe'),
            ('~/map', 'dlio/map_node/map')
        ]
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', os.path.join(package_dir, 'launch', 'dlio.rviz')],
        condition=lambda context: LaunchConfiguration('rviz').perform(context) == 'true'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(rviz_arg)
    
    # Add the nodes
    ld.add_action(dlio_odom_node)
    ld.add_action(dlio_map_node)
    ld.add_action(rviz_node)
    
    return ld