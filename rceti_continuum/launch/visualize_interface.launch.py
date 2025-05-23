import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rceti_continuum_path = get_package_share_directory('rceti_continuum')

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="False", description="Enable GUI"),

        Node(
            package="rceti_continuum",
            executable="continuum_interface_node",
            name="continuum_interface_node",
            output="screen",
            parameters=[{
                "number_of_segments": 2
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", os.path.join(rceti_continuum_path, "urdf", "continuum.rviz")],
            output="screen"
        ),
    ])
