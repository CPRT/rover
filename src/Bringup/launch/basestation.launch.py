import os

import launch_ros.actions
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="joy", executable="joy_node", name="joystick"
            ),
            launch_ros.actions.Node(
                package="gps",
                executable="rtcm_pub_node",
                name="gps_basestation_node",
                parameters=[
                    {"TimingMode": 1},  # Survey In mode
                    {"MinTime": 600},  # Survey in time (s)
                    {"MinAcc": 2.0},  # Survey In minimum accuracy (m)
                    {"Freq": 5.0},  # Publish rate (hz)
                    {"Baudrate": 9600},
                    {"Device": "/dev/ttyACM0"},
                ],
            ),
        ]
    )
