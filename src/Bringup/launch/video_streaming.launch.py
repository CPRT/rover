from launch import LaunchDescription
import os


def generate_launch_description():
    launch_files = [
        ("camera_streaming", "webRTC.launch.py"),
    ]
    return LaunchDescription(get_included_launch_descriptions(launch_files))
