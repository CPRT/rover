from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
import os


def get_included_launch_descriptions(launch_files):
    included_launches = []
    for pkg, file in launch_files:
        pkg_share = FindPackageShare(pkg).find(pkg)
        file_path = os.path.join(pkg_share, "launch", file)
        included_launches.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(file_path))
        )
    return included_launches


def generate_launch_description():
    SetEnvironmentVariable("ROS_LOG_LEVEL", "WARN")
    launch_files = [
        ("joystick_control", "controller.launch.py"),
        ("servo_pkg", "usb_servo_launch.launch.py"),
    ]
    return LaunchDescription(get_included_launch_descriptions(launch_files))
