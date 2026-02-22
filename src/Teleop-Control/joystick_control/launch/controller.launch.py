import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
import os
from ament_index_python.packages import get_package_share_directory


def get_joy_id(dev_path):
    """Checks for a symlink and returns the integer ID, or a default."""
    if os.path.exists(dev_path):
        # Resolves /dev/joy_a -> /dev/input/jsX
        real_path = os.path.realpath(dev_path)
        # Pulls the digits out of the path string
        try:
            print(f"Found symlink {dev_path} -> {real_path}")
            return int("".join(filter(str.isdigit, real_path)))
        except ValueError:
            print(f"ERROR: Device {real_path} not a event joystick path")
            return -1
    else:
        print(f"ERROR: Symlink {dev_path} does not exist")
        return -1


def generate_launch_description():
    pkg_joystick_control = get_package_share_directory("joystick_control")
    parameters_file = os.path.join(pkg_joystick_control, "pxn.yaml")
    # Detect IDs dynamically
    id_arm = get_joy_id("/dev/input/by-id/usb-LiteStar_PXN-2113_Pro-joystick")
    id_drive = get_joy_id("/dev/input/by-id/usb-Thrustmaster_T.1600M-joystick")

    ld = LaunchDescription()

    if id_drive != -1:
        ld.add_action(
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node_a",
                parameters=[
                    {
                        "device_id": id_drive,
                        "deadzone": 0.05,
                    }
                ],
                remappings=[("/joy", "/drive/joy")],
            )
        )
    if id_arm != -1:
        ld.add_action(
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node_arm",
                parameters=[
                    {
                        "device_id": id_arm,
                        "deadzone": 0.05,
                    }
                ],
                remappings=[("/joy", "/arm/joy")],
            ),
        )
    ld.add_action(
        Node(
            package="joystick_control",
            executable="arm",
            name="arm_teleop_node",
            parameters=[parameters_file],
            remappings=[("/joy", "/arm/joy")],
        ),
    )
    ld.add_action(
        Node(
            package="joystick_control",
            executable="drive",
            name="drive_teleop_node",
            parameters=[parameters_file],
            remappings=[("/joy", "/drive/joy")],
        ),
    )
    return ld
