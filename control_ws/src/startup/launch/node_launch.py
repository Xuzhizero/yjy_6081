from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file_path_mode = os.path.expanduser('~/mode_node.yaml')
    config_file_path_controller = os.path.expanduser('~/controller_node.yaml')
    return LaunchDescription([
        Node(
            package='data',
            executable='data',
            name='data_node',
            output='screen',
            respawn=True,  # 启用自动重启‌:ml-citation{ref="1,2" data="citationList"}
            respawn_delay=2  # 重启延迟2秒‌:ml-citation{ref="1" data="citationList"}
        ),
        Node(
            package='control',
            executable='mode',
            name='mode_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
            parameters=[config_file_path_mode]
        ),
        Node(
            package='control',
            executable='controller',
            name='controller_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
            parameters=[config_file_path_controller]
        ),
        # Node(
        #     package='rudder',
        #     executable='rudder',
        #     name='rudder_node',
        #     output='screen',
        #     respawn=True,
        #     respawn_delay=2
        # ),
    ])