import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    share_dir = get_package_share_directory("elevation_mapping")
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "--display-config",
                    os.path.join(share_dir, "rviz2", "zed2i.rviz"),
                ],
                output="screen",
            )
        ]
    )
