from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_node = Node(
        package='arduino', executable='controller_node', name='controller_node',
    )

    return LaunchDescription([
        controller_node
    ])