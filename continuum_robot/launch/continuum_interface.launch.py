from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='continuum_robot',
            executable='continuum_interface_node',
            name='continuum_interface_node',
            output='screen',
            parameters=[{
                'number_of_segments': 2  # adjust as needed
            }]
        )
    ])
