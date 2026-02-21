import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common.launch import RewrittenYaml


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

    set_gravity_as_origin = LaunchConfiguration("set_gravity_as_origin")
    set_gravity_as_origin_launch_arg = DeclareLaunchArgument(
        "set_gravity_as_origin",
        default_value="false",
        description="If true, align the positional tracking world to IMU gravity measurement.",
    )

    publish_tf = LaunchConfiguration("publish_tf")
    publish_tf_launch_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="If true, ZED publishes odom->base_link TF. Enable for SVO playback without EKF.",
    )

    publish_map_tf = LaunchConfiguration("publish_map_tf")
    publish_map_tf_launch_arg = DeclareLaunchArgument(
        "publish_map_tf",
        default_value="false",
        description="If true, ZED publishes map->odom TF. Enable for SVO playback without EKF.",
    )

    # Rewrite the ZED parameters to override set_gravity_as_origin
    param_substitutions = {
        "set_gravity_as_origin": set_gravity_as_origin,
    }

    configured_params = RewrittenYaml(
        source_file=zed_override_params_filepath,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    zed_launch_arguments = {
        "ros_params_override_path": configured_params,
        "camera_model": "zed2i",
        "publish_urdf": "true",
        "publish_tf": publish_tf,
        "publish_map_tf": publish_map_tf,
        "svo_path": svo_path,
    }

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_launch_filepath]),
        launch_arguments=zed_launch_arguments.items(),
    )

    return LaunchDescription(
        [
            svo_path_launch_arg,
            set_gravity_as_origin_launch_arg,
            publish_tf_launch_arg,
            publish_map_tf_launch_arg,
            zed_launch,
        ]
    )
