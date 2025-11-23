from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def get_included_launch_descriptions(launch_files):
    included_launches = []
    for pkg, file in launch_files:
        pkg_share = FindPackageShare(pkg).find(pkg)
        file_path = os.path.join(pkg_share, "launch", file)

        if file.endswith(".py"):
            source = PythonLaunchDescriptionSource(file_path)
        elif file.endswith(".xml"):
            source = XMLLaunchDescriptionSource(file_path)
        else:
            raise ValueError(f"Unsupported launch file format: {file}")

        included_launches.append(IncludeLaunchDescription(source))
    return included_launches


def generate_launch_description():
    launch_files = [
        ("joystick_control", "controller.launch.py"),
        ("servo_pkg", "usb_servo_launch.launch.py"),
        ("rosbridge_server", "rosbridge_websocket_launch.xml"),
    ]
    return LaunchDescription(get_included_launch_descriptions(launch_files))
