import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
import os
from ament_index_python.packages import get_package_share_directory

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
    arm_dev = "/dev/input/by-id/usb-LiteStar_PXN-2113_Pro-joystick"
    drive_dev = find_ps5()

    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node_a",
            parameters=[
                {
                    "dev": drive_dev,
                    "deadzone": 0.05,
                }
            ],
            remappings=[("/joy", "/drive/joy")],
        )
    )
    ld.add_action(
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node_arm",
            parameters=[
                {
                    "dev": arm_dev,
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
