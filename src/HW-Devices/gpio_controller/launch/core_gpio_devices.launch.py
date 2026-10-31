import launch
import launch_ros.actions
import os


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="headlights",
                name="headlight_node",
            ),
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="mast_esp",
                name="mast_esp_node",
            ),
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="pdb_rails",
                name="pdb_rails_node",
            ),
        ]
    )
