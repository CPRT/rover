import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_servo = get_package_share_directory("servo_pkg")
    child_params = os.path.join(pkg_servo, "config", "usb_controller.yaml")
    parent_params = os.path.join(pkg_servo, "config", "parent_config.yaml")
    # if you wanted to run client alongside
    # client_params = os.path.join(pkg_servo, "config", "servo_client.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="servo_pkg",
                executable="USB_Servo",
                name="USB_Servo_node",
                parameters=[parent_params, child_params],
            )
        ]
    )
