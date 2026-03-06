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


def find_ps5():
    with open("/proc/bus/input/devices") as f:
        device_block = ""
        for line in f:
            if line.strip() == "":
                if "Wireless Controller" in device_block:
                    for part in device_block.split():
                        if part.startswith("js"):
                            print(f"Found PS5 controller at {part}")
                            return int(part.replace("js", ""))
                device_block = ""
            else:
                device_block += line
    print(f"ERROR: PS5 controller not found in /proc/bus/input/devices")
    return -1


def generate_launch_description():
    pkg_joystick_control = get_package_share_directory("joystick_control")
    parameters_file = os.path.join(pkg_joystick_control, "pxn.yaml")
    # Detect IDs dynamically
    id_arm = get_joy_id("/dev/input/by-id/usb-LiteStar_PXN-2113_Pro-joystick")
    # id_drive = get_joy_id("/dev/input/by-id/usb-Thrustmaster_T.16000M-joystick")
    # id_drive = get_joy_id("/dev/input/by-id/bluetooth-Sony_Interactive_Entertainment_DualSense_Wireless_Controller-if03-joystick")

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node_a",
                parameters=[
                    {
                        "dev": find_ps5(),
                        "deadzone": 0.05,
                    }
                ],
                remappings=[("/joy", "/drive/joy")],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node_arm",
                condition=IfCondition(PythonExpression([str(id_arm), " >= 0"])),
                parameters=[
                    {
                        "device_id": id_arm,
                        "deadzone": 0.05,
                    }
                ],
                remappings=[("/joy", "/arm/joy")],
            ),
            Node(
                package="joystick_control",
                executable="arm",
                name="arm_teleop_node",
                parameters=[parameters_file],
                remappings=[("/joy", "/arm/joy")],
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
