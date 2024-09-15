from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    arduino_node = Node(
        package='arduino', executable='arduino_node', name='arduino_node',
    )

    return LaunchDescription([
        arduino_node
    ])