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

    camera_node = Node(
        package='realsense2_camera', executable='rs_launch.py', name='camera_node',
        parameters=[params],
    )

    return LaunchDescription([
        camera_node
    ])