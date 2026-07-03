import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    linktrack_aoa_param_file=os.path.join(get_package_share_directory('nlink_parser'),'params','linktrack_aoa_init_params.yaml')
    ld = LaunchDescription()
    linktrack_aoa_node = Node(
        name="linktrack_aoa",
        package="nlink_parser",
        executable="linktrack_aoa",
        output="screen",
        parameters=[{"pub_frequency":5.0}],
        arguments=[linktrack_aoa_param_file]
    )
    ld.add_action(linktrack_aoa_node)


    return ld
