import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('metasepia_core'),
        'config',
        'camera.json'
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py"
            )
        ]),
        launch_arguments={
            'enable_infra' : "false",
            'enable_depth' : "false",
            'enable_color' : "true",
            'rgb_camera.color_profile' : "424x240x15",
        }.items()
    )

    return LaunchDescription([
        camera_node
    ])