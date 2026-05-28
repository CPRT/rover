import os
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    load_config = LaunchConfiguration("load_config", default="false")

    return launch.LaunchDescription(
        [
            # Declare a boolean launch argument to control loading config on startup
            DeclareLaunchArgument(
                "load_config",
                default_value="false",
                description="If true, load GPS config on startup",
            ),
            launch_ros.actions.Node(
                package="gps",
                executable="heading_pub_node",
                name="gps_heading_node",
                output="screen",
                parameters=[
                    {"frame_id": "gps_link"},
                    {"Freq": 5.0},  # Publish rate (hz)
                    {"Baudrate": 115200},
                    {
                        "Device": "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D30EFLJN-if00-port0"
                    },
                ],
                remappings=[("heading", "gps/heading")],
            ),
            # Optionally run the standalone config script when requested
            # ExecuteProcess(
            #     cmd=[
            #         "python3",
            #         os.path.join(
            #             get_package_share_directory("gps"), "config", "fr_heading.py"
            #         ),
            #     ],
            #     condition=IfCondition(load_config),
            # ),
        ]
    )
