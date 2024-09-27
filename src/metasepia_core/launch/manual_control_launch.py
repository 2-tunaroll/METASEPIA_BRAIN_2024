import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    arduino_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('metasepia_core'),
                'launch',
                'arduino_launch.py'
            )
        ])
    )

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('metasepia_core'),
                'launch',
                'joy_launch.py'
            )
        ])
    )

    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('metasepia_core'),
                'launch',
                'controller_launch.py'
            )
        ])
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("metasepia_core"),
                "launch",
                "camera_launch.py"
            )
        ])
    )

    rosbridge_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("metasepia_core"),
                "launch",
                "dashboard_launch.py"
            )
        ])
    )


    return LaunchDescription([
        # arduino_node,
        joy_node,
        camera_node,
        rosbridge_node,
        controller_node
    ])