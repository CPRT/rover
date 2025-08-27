import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    zed_launch_filepath = os.path.join(
        get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py"
    )
    zed_override_params_filepath = os.path.join(
        get_package_share_directory("navigation"),
        "config",
        "zed",
        "zed_override_params.yaml",
    )

    # Launch configuration variables
    svo_path = LaunchConfiguration("svo_path")
    svo_path_launch_arg = DeclareLaunchArgument(
        "svo_path",
        default_value=TextSubstitution(text="live"),
        description="Path to an input SVO file.",
    )

    zed_launch_arguments = {
        "ros_params_override_path": zed_override_params_filepath,
        "camera_model": "zed2i",
        "publish_urdf": "true",
        "publish_tf": "false",
        "publish_map_tf": "false",
        "svo_path": svo_path,
    }

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_launch_filepath]),
        launch_arguments=zed_launch_arguments.items(),
    )

    return LaunchDescription([svo_path_launch_arg, zed_launch])
