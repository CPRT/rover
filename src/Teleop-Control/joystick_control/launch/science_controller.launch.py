import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def get_joy_id(dev_path, default_id):
    """Checks for a symlink and returns the integer ID, or a default."""
    if os.path.exists(dev_path):
        # Resolves /dev/joy_a -> /dev/input/jsX
        real_path = os.path.realpath(dev_path)
        # Pulls the digits out of the path string
        try:
            print(f"Found symlink {dev_path} -> {real_path}")
            return int("".join(filter(str.isdigit, real_path)))
        except ValueError:
            print(
                f"Could not parse joystick ID from {real_path}, using default {default_id}"
            )
            return default_id
    else:
        print(f"Symlink {dev_path} does not exist, using default ID {default_id}")
        return default_id


def generate_launch_description():
    pkg_joystick_control = get_package_share_directory("joystick_control")
    parameters_file = os.path.join(pkg_joystick_control, "pxn.yaml")
    # Detect IDs dynamically
    # We use 0 and 1 as defaults if the symlinks aren't found
    id_a = get_joy_id("/dev/input/by-id/usb-LiteStar_PXN-2113_Pro-joystick", 1)
    id_b = get_joy_id("/dev/input/by-id/usb-Thrustmaster_T.1600M-joystick", 0)

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node_a",
                parameters=[
                    {
                        "device_id": id_a,
                        "deadzone": 0.05,
                    }
                ],
                remappings=[("/joy", "/drive/joy")],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node_b",
                parameters=[
                    {
                        "device_id": id_b,
                        "deadzone": 0.05,
                    }
                ],
                remappings=[("/joy", "/science/joy")],
            ),
            Node(
                package="joystick_control",
                executable="science",
                name="science_teleop_node",
                parameters=[parameters_file],
                remappings=[("/joy", "/science/joy")],
            ),
            Node(
                package="joystick_control",
                executable="drive",
                name="drive_teleop_node",
                parameters=[parameters_file],
                remappings=[("/joy", "/drive/joy")],
            ),
        ]
    )
