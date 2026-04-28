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
                executable="gps_base_pub_node",
                name="gps_base_pub_node",
                parameters=[
                    {
                        "Device": "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
                    },
                    {"Baudrate": 115200},
                    {"Freq": 10.0},  # Publish rate (hz)
                    {"SvinMindur": 0},  # Survey in time (s)
                    {"SvinMinAccDur": 10_000},  # Survey in time (s)
                    {"QueueDepth": 10},
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
