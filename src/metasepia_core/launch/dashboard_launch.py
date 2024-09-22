import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    rosbridge_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml"
            )
        ]),
    )

    return LaunchDescription([
        rosbridge_node
    ])

