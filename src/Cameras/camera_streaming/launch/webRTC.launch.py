import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import launch


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("camera_streaming"), "config")

    params_file = os.path.join(config_dir, "webrtc.yaml")

    webrtc_node = launch_ros.actions.Node(
        package="camera_streaming",
        executable="webrtc_node",
        output="log",
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", "Info"],
    )

    return launch.LaunchDescription([webrtc_node])
