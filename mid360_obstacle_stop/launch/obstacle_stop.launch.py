"""启动 mid360_obstacle_stop 节点。

用法:
    ros2 launch mid360_obstacle_stop obstacle_stop.launch.py
    ros2 launch mid360_obstacle_stop obstacle_stop.launch.py params_file:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mid360_obstacle_stop')
    default_params_file = os.path.join(package_share, 'config', 'obstacle_stop.yaml')

    params_file = LaunchConfiguration('params_file')
    node_name = LaunchConfiguration('node_name')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='障碍停车节点的参数文件 (config/obstacle_stop.yaml)',
    )
    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='mid360_obstacle_stop',
        description='节点名（必须与参数文件中的顶层 key 一致）',
    )

    obstacle_stop_node = Node(
        package='mid360_obstacle_stop',
        executable='obstacle_stop_node',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_params_file,
        declare_node_name,
        obstacle_stop_node,
    ])
