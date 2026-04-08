from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def find_ps5():
    try:
        with open("/proc/bus/input/devices", "r") as f:
            device_block = ""
            for line in f:
                if line.strip() == "":
                    if "Wireless Controller" in device_block:
                        # Find the handlers line, e.g., H: Handlers=event19 js0
                        for part in device_block.split():
                            if part.startswith("js"):
                                # Return the full path string, not just the number
                                dev_path = f"/dev/input/{part.strip()}"
                                print(f"Found PS5 controller at {dev_path}")
                                return dev_path
                    device_block = ""
                else:
                    device_block += line
    except Exception as e:
        print(f"Error reading devices: {e}")

    print("ERROR: PS5 controller not found. Defaulting to /dev/input/js0")
    return "/dev/input/js0"


def generate_launch_description():
    pkg_joystick_control = get_package_share_directory("joystick_control")
    parameters_file = os.path.join(pkg_joystick_control, "pxn.yaml")
    # Detect IDs dynamically
    arm_dev = "/dev/input/by-id/usb-LiteStar_PXN-2113_Pro-joystick"
    drive_dev = "/dev/input/by-id/usb-Thrustmaster_T.16000M-joystick"

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
