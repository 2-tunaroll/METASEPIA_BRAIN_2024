from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(get_package_share_directory('arduino'),'config','joystick.yaml')

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[params],
    )

    arduino_node = Node(
        package='arduino', executable='arduino_node', name='arduino_node',
    )

    return LaunchDescription([
        joy_node,
        arduino_node
    ])