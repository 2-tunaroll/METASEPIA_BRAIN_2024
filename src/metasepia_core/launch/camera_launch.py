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
            'enable_depth' : "true",
            'depth_module.depth_profile' : "256x144x90",
            'enable_color' : "true",
            'rgb_camera.color_profile' : "424x240x15",
            '/camera/color/image_raw/compressed/format' : "jpeg",  # Converted XML param
            '/camera/color/image_raw/compressed/jpeg_quality' : "23",  # Converted XML param
            # 'pointcloud.enable' : "true"
        }.items()
    )

    return LaunchDescription([
        camera_node
    ])