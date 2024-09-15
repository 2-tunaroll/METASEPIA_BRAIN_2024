from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('metasepia_core'),
        'config',
        'metasepia.yaml'
    )

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[params],
    )

    return LaunchDescription([
        joy_node
    ])