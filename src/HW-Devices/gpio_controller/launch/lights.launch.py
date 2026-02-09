import launch
import launch_ros.actions
import os


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="lights",
                name="led_strip_nodes",
            ),
        ]
    )
