import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    # Get package directories
    pkg_navigation = get_package_share_directory("navigation")
    pkg_localization = get_package_share_directory("localization")
    pkg_aruco = get_package_share_directory("aruco_localizer")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    svo_path = LaunchConfiguration("svo_path")

    # Determine if we're playing back SVO or running live
    svo_path_value = svo_path.perform(context)
    is_svo_playback = svo_path_value != "live"

    # Configure based on SVO playback mode
    if is_svo_playback:
        # SVO Playback Mode: Let ZED handle all TF publishing directly
        set_gravity_as_origin = "true"
        publish_tf = "true"
        publish_map_tf = "true"
        launch_localization = "false"
        launch_aruco = False
        traversability_profile = "zed_only_svo"
    else:
        # Live Mode: Use EKF with multiple sensor fusion
        set_gravity_as_origin = "false"
        publish_tf = "false"
        publish_map_tf = "false"
        launch_localization = "true"
        launch_aruco = True
        traversability_profile = "zed_only"

    # Launch SLAM launch file
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "svo_path": svo_path,
            "set_gravity_as_origin": set_gravity_as_origin,
            "publish_tf": publish_tf,
            "publish_map_tf": publish_map_tf,
            "traversability_profile": traversability_profile,
            "launch_localization": launch_localization,
            "launch_ouster": "false",
        }.items(),
    )

    # Launch localization only in live mode
    if launch_localization == "true":
        localization_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_localization, "launch", "localization.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "ekf_profile": "indoor_aruco_boards",
                "launch_gps": "false",
            }.items(),
        )
    else:
        localization_cmd = None

    # Launch ArUco board detector only in live mode
    if launch_aruco:
        aruco_detector_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_aruco, "launch", "aruco_board_detector.launch.py")
            ),
        )
    else:
        aruco_detector_cmd = None

    # Static transform from zed_camera_link to base_link (only in SVO playback mode)
    if is_svo_playback:
        base_link_static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_to_base_tf",
            arguments=[
                "--x",
                "-0.489",
                "--y",
                "0.03",
                "--z",
                "-0.284",
                "--roll",
                "0.0",
                "--pitch",
                "-0.33",
                "--yaw",
                "0.0",
                "--frame-id",
                "zed_camera_link",
                "--child-frame-id",
                "base_link",
            ],
        )
    else:
        base_link_static_tf = None

    # Return only the commands that are not None
    commands = [slam_cmd]
    if localization_cmd is not None:
        commands.append(localization_cmd)
    if aruco_detector_cmd is not None:
        commands.append(aruco_detector_cmd)
    if base_link_static_tf is not None:
        commands.append(base_link_static_tf)

    return commands


def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time if true"
    )

    declare_svo_path = DeclareLaunchArgument(
        "svo_path", default_value="live", description="Path to an input SVO file."
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_svo_path,
            OpaqueFunction(function=launch_setup),
        ]
    )
