import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


# Make launch configeration, defult false, launch talon on rover. Defult True for xbox
def generate_launch_description():
    """Generate launch description with multiple components."""
    pkg_drive = get_package_share_directory("drive_cpp")
    parameters_file = os.path.join(pkg_drive, "config", "pxn.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="joystick_control",
                executable="flightstick_control",
                name="flightstick_control",
                output="screen",
                parameters=[parameters_file],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="lights",
                name="lights",
                output="screen",
            ),
        ]
    )
