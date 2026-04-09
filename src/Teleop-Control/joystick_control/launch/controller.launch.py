<<<<<<< HEAD
=======
import launch
import launch_ros.actions
>>>>>>> cc5fb9f (feat: implement modeless teleop and move_group action server)
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


<<<<<<< HEAD
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

=======
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

>>>>>>> cc5fb9f (feat: implement modeless teleop and move_group action server)

def generate_launch_description():
    pkg_joystick_control = get_package_share_directory("joystick_control")
    parameters_file = os.path.join(pkg_joystick_control, "pxn.yaml")
<<<<<<< HEAD
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
=======
    positions_file = os.path.join(pkg_joystick_control, "positions.yaml")
    # Detect IDs dynamically
    # We use 0 and 1 as defaults if the symlinks aren't found
    id_a = get_joy_id("/dev/input/by-id/usb-LiteStar_PXN-2113_Pro-joystick", 0)
    id_b = get_joy_id("/dev/input/by-id/usb-Thrustmaster_T.1600M-joystick", 1)

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
            Node(
                package="joystick_control",
                executable="movegroup_controller",
                name="movegroup_control_action_server",
                parameters=[parameters_file],
            ),
        ]
>>>>>>> cc5fb9f (feat: implement modeless teleop and move_group action server)
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
