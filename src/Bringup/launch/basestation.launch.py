import os

import launch_ros.actions
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gps",
                executable="rtcm_pub_node",
                name="gps_basestation_node",
                parameters=[
                    {"TimingMode": 1},
                    {"MinTime": 600},
                    {"MinAcc": 2.0},
                    {"Freq": 5.0},
                    {"Baudrate": 9600},
                    {
                        "Device": "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("joystick_control"),
                        "launch",
                        "controller.launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("video_streaming"),
                        "launch",
                        "rtp_client.launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    )
                )
            ),
            launch_ros.actions.Node(
                package="ros_roboclaw",
                executable="roboclaw_node",
                name="antenna_roboclaw_node",
                parameters=[
                    {
                        "port": "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x30A-if00"
                    },
                ],
                remappings=[("/robo_duty_cycle", "/antenna_control")],
            ),
        ]
    )
