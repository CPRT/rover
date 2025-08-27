from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="computer_vision",
                executable="zed_aruco_detector_node",
                name="zed_aruco_detector_node",
                output="screen",
            )
        ]
    )
