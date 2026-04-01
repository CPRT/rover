import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("gps"), "config")

    params_file = os.path.join(config_dir, "gps.yaml")

    load_config = LaunchConfiguration("load_config", default="false")

    heading_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gps"), "launch", "heading.launch.py"
            )
        ),
        launch_arguments={"load_config": load_config}.items(),
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "load_config",
                default_value="false",
                description="If true, load GPS config on startup",
            ),
            launch_ros.actions.Node(
                package="ublox_gps",
                executable="ublox_gps_node",
                output="both",
                parameters=[
                    params_file,
                ],
                remappings=[
                    ("ublox_gps_node/fix", "gps/fix"),
                    ("/navheading", "gps/heading"),
                ],
            ),
            # Optionally run the rover config script when requested
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(
                        get_package_share_directory("gps"), "config", "fr_rover.py"
                    ),
                ],
                condition=IfCondition(load_config),
            ),
            heading_cmd,
        ]
    )


# If you run this with load_config true twice in a row then it will trigger, [ublox_gps_node-1] [ERROR] [1768684861.161342843] [gps_rover_node]: U-Blox ASIO input buffer read error: End of file, 0
# but if you run anything else on the serial port in between it seems to work.
