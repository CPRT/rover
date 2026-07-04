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
                    {"Freq": 5.0},  # Publish rate (hz)
                    {"SvinMindur": 300},  # Survey in time (s)
                    {"SvinMinAccDur": 10_000},  # Survey in accuracy (mm)
                    {"QueueDepth": 10},
                ],
            ),
            launch_ros.actions.Node(
                package="gps",
                executable="antenna_pointing_node",
                name="antenna_pointing_node",
                parameters=[
                    {"Freq": 5},
                ],
            ),
            launch_ros.actions.Node(
                package="ros_roboclaw",
                executable="antenna_roboclaw_node",
                name="antenna_roboclaw_node",
                parameters=[
                    {
                        "Port": "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x30A-if00"
                    },
                    {"baudrate": 115200},
                    {"MaxSpeed": 1000},  # Max Speed (enc counts)
                    {"Accel": 500},  # Max accel (enc Counts)
                    {
                        "EncReadFreq": 10.0  # encoder reads for up to date comparisons between
                    },
                    {"Address": 0x80},  # this is the default for roboclaw
                    {
                        "CountsPerRev": 8192  # based on 4096 encoder resolution and 2:1 gear ratio
                    },
                ],
                remappings=[
                    ("/roboclaw_desired_position", "/antenna/target_bearing"),
                    ("/roboclaw_actual_position", "/antenna/bearing"),
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
