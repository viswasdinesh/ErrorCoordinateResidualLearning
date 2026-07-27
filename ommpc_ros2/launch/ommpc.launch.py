import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Retrieve the package path
    pkg_share = get_package_share_directory('ommpc_ros2')
    
    # Path to parameter YAML
    # First check if package share contains it, otherwise point to source
    params_yaml = os.path.join(pkg_share, 'config', 'params.yaml')
    if not os.path.exists(params_yaml):
        params_yaml = '/home/dinesh/ecrl_ws/src/ommpc_ros2/config/params.yaml'

    # OMMPC controller node
    ommpc_node = Node(
        package='ommpc_ros2',
        executable='ommpc_node',
        name='ommpc_node',
        output='screen',
        parameters=[params_yaml]
    )

    # ECRL data collector node
    data_collector_node = Node(
        package='ommpc_ros2',
        executable='ecrl_data_collector_node',
        name='ecrl_data_collector_node',
        output='screen',
        parameters=[
            {'csv_filepath': '/home/dinesh/ecrl_datasets_v2/polynomial1.csv'},
        ]
    )

    return LaunchDescription([
        ommpc_node,
        data_collector_node
    ])
