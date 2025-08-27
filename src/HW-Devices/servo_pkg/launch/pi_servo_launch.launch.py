import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_servo = get_package_share_directory("servo_pkg")
    parameters_file = os.path.join(pkg_servo, "config", "pi_controller.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="servo_pkg",
                executable="pi_Servo",
                name="pi_Servo_node",
                parameters=[parameters_file],
                remappings=[("servo_service", "science_servo_service")],
            ),
            # for example client
            # launch_ros.actions.Node(
            #     package="servo_pkg",
            #     executable="servo_client",
            #     name="servo_client_node",
            # ),
        ]
    )
