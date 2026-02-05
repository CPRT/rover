import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition


def generate_launch_description():
    load_config = LaunchConfiguration("load_config", default="false")

    return launch.LaunchDescription(
        [
            # Declare launch argument to control loading configuration on startup
            DeclareLaunchArgument(
                "load_config",
                default_value="false",
                description="If true, load basestation config on startup",
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
                    {"Device": "/dev/ttyUSB0"},
                ],
            ),
            # Optionally run the standalone basestation config script when requested
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(
                        get_package_share_directory("gps"),
                        "config",
                        "fr_basestation.py",
                    ),
                ],
                condition=IfCondition(load_config),
            ),
        ]
    )
