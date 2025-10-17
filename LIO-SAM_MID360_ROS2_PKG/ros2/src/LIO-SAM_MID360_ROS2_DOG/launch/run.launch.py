import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

ONLINE_LIDAR = False

def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_path = LaunchConfiguration('bag_path')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    ndt_params_declare = DeclareLaunchArgument(
        'ndt_params_file',
        default_value=os.path.join(
            share_dir, 'config', 'ndt_params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    # rosbag_record = ExecuteProcess(
    #         cmd=['ros2', 'bag', 'record', '-o', '/home/ywj/projects/dataset/robot/livox_record_new/', 
    #              '/livox/lidar', '/livox/imu',],
    #         output='screen'
    #     )
        
    bag_path_declare = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/ywj/projects/dataset/robot/livox_record_new/', # 使用裁切后的数据作为默认路径
        description='Path to the bag file to play (can be cropped data)'
    )

    # 移除静态TF发布器，让LIO-SAM发布动态map->odom变换
    # static_transform_publisher_node = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
    #         output='screen'
    #         )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path]),
            'use_sim_time': use_sim_time
        }]
    )
    lio_sam_imuPreintegration_node = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[parameter_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    lio_sam_imageProjection_node = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[parameter_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    lio_sam_featureExtraction_node = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[parameter_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    lio_sam_mapOptimization_node = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[parameter_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',                   # 地图坐标系
            'sensor_model/max_range': 10.0,      # 最大感测距离
            'sensor_model/min_range': 0.8,       # 最小感测距离
            'sensor_model/insert_free_space': True,
            'resolution': 0.05,                  # OctoMap 分辨率（5cm）
            'occupancy_min_z': -1.0,             # 投影高度下限
            'occupancy_max_z': 1.5,              # 投影高度上限
            'publish_2d_map': True,               # 输出2D occupancy grid
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cloud_in', '/lio_sam/mapping/cloud_registered')  # 输入点云
        ]
    )


    
    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen'
    # )

    
    if ONLINE_LIDAR:
        return LaunchDescription([
            params_declare,
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (bag) time'
            ),
            # rosbag_record,
            # static_transform_publisher_node,  # 移除静态TF
            robot_state_publisher_node,
            lio_sam_imuPreintegration_node,
            lio_sam_imageProjection_node,
            lio_sam_featureExtraction_node,
            lio_sam_mapOptimization_node,
            octomap_server_node,
            # rviz2_node
        ])
    else:
        return LaunchDescription([
            params_declare,
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (bag) time'
            ),
            bag_path_declare,
            # 静态变换发布节点
            # static_transform_publisher_node,  # 移除静态TF，让LIO-SAM发布动态map->odom
            robot_state_publisher_node,
            lio_sam_imuPreintegration_node,
            lio_sam_imageProjection_node,
            lio_sam_featureExtraction_node,
            lio_sam_mapOptimization_node,
            octomap_server_node,
            # rviz2_node,
            # Bag 数据播放，添加QoS配置覆盖、开始时间和时钟参数
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_path, '--qos-profile-overrides-path', '/home/ywj/projects/dataset/reliability_override.yaml', '--clock', '--rate', '1.0'],
                name='rosbag_player',
                output='screen'
            )
        ])
