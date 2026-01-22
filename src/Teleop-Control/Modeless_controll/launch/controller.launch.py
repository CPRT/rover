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
            return int("".join(filter(str.isdigit, real_path)))
        except ValueError:
            launch.actions.LogInfo(f"Could not parse joystick ID from {real_path}, using default {default_id}")
            return default_id
    return default_id


def generate_launch_description():
    pkg_drive = get_package_share_directory("drive_cpp")
    parameters_file = os.path.join(pkg_drive, "config", "pxn.yaml")
    # Detect IDs dynamically
    # We use 0 and 1 as defaults if the symlinks aren't found
    id_a = get_joy_id("/dev/joy_a", 1)
    id_b = get_joy_id("/dev/joy_b", 0)

    return LaunchDescription(
        [
            # Controller A (Top/Left)
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
                remappings=[("/joy", "/controller_a/joy")],
            ),
            # Controller B (Bottom/Right)
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
                remappings=[("/joy", "/controller_b/joy")],
            ),
            Node(
                package="Modeless_controll",
                executable="armManualModeless",
                name="armManualModeless",
                parameters=[parameters_file],
            ),
            Node(
                package="Modeless_controll",
                executable="driveModeless",
                name="driveModeless",
                parameters=[parameters_file],
            ),
            Node(
                package="gpio_controller",
                executable="lights",
                name="lights",
                output="screen",
            ),
        ]
    )
