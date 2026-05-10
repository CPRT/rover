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
                executable="distance_sensor",
                name="distance_sensor_node",
            ),
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="pdb_rails",
                name="pdb_rails_node",
            ),
        ]
    )
