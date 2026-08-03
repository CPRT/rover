import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rosbridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        parameters=[
            {"default_call_service_timeout": 60.0},
            {"max_message_size": 10000000},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(rosbridge_websocket)
    return ld
