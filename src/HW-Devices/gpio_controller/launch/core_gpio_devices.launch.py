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
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="mast_esp",
                name="mast_esp_node",
            ),
            # launch_ros.actions.Node(
            #     package="gpio_controller",
            #     executable="pdb_rails",
            #     name="pdb_rails_node",
            # ),
        ]
    )
