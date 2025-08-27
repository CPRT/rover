import os

import ament_index_python.packages
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("gps"), "config")

    params_file = os.path.join(config_dir, "gps.yaml")

    Rover_config_file = os.path.join(config_dir, "Rover_config.ubx")
    Rover_heading = (
        "ubxload --port /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 --baudrate 9600 --infile "
        + Rover_config_file
    )
    # TODO: Uncomment the line below to configure the rover GPS on startup
    # os.system(Rover_heading)

    ublox_remappings = [
        ("ublox_gps_node/fix", "gps/fix"),
        ("/navheading", "gps/heading"),
    ]

    ublox_gps_node = launch_ros.actions.Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="both",
        remappings=ublox_remappings,
        parameters=[params_file],
    )
    heading_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gps"), "launch", "heading.launch.py"
            )
        )
    )

    return launch.LaunchDescription([ublox_gps_node, heading_cmd])
