import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_servo = get_package_share_directory("servo_pkg")
    parent_params = os.path.join(pkg_servo, "config", "parent_config.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="servo_pkg",
                executable="pdb_controller",
                name="pdb_servo",
                parameters=[parent_params],
                remappings=[
                    ("~/pan", "/pan"),
                    ("~/tilt", "/tilt"),
                ],
            )
        ]
    )
