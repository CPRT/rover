import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package directories
    pkg_navigation = get_package_share_directory("navigation")
    pkg_localization = get_package_share_directory("localization")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    svo_path = LaunchConfiguration("svo_path")
    set_gravity_as_origin = LaunchConfiguration("set_gravity_as_origin")
    publish_tf = LaunchConfiguration("publish_tf")
    publish_map_tf = LaunchConfiguration("publish_map_tf")
    traversability_profile = LaunchConfiguration("traversability_profile")
    launch_traversability = LaunchConfiguration("launch_traversability")
    launch_zed = LaunchConfiguration("launch_zed")
    launch_ouster = LaunchConfiguration("launch_ouster")
    launch_localization = LaunchConfiguration("launch_localization")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_traversability = DeclareLaunchArgument(
        "launch_traversability", default_value="True"
    )
    declare_localization = DeclareLaunchArgument(
        "launch_localization", default_value="True"
    )
    declare_zed = DeclareLaunchArgument("launch_zed", default_value="True")
    declare_ouster = DeclareLaunchArgument("launch_ouster", default_value="False")
    declare_svo_path = DeclareLaunchArgument(
        "svo_path",
        default_value=TextSubstitution(text="live"),
        description="Path to an input SVO file.",
    )
    declare_set_gravity_as_origin = DeclareLaunchArgument(
        "set_gravity_as_origin",
        default_value="false",
        description="If true, align the positional tracking world to IMU gravity measurement.",
    )
    declare_publish_tf = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="If true, ZED publishes odom->base_link TF.",
    )
    declare_publish_map_tf = DeclareLaunchArgument(
        "publish_map_tf",
        default_value="false",
        description="If true, ZED publishes map->odom TF.",
    )
    declare_traversability_profile = DeclareLaunchArgument(
        "traversability_profile",
        default_value="zed_only",
        description="Elevation mapping profile to use.",
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "localization.launch.py")
        ),
        condition=IfCondition(launch_localization),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    zed_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "zed.launch.py")
        ),
        condition=IfCondition(launch_zed),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "svo_path": svo_path,
            "set_gravity_as_origin": set_gravity_as_origin,
            "publish_tf": publish_tf,
            "publish_map_tf": publish_map_tf,
        }.items(),
    )

    traversability_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "traversability_gridmap.launch.py")
        ),
        condition=IfCondition(launch_traversability),
        launch_arguments={
            "profile": traversability_profile,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Include Ouster lidar launch if enabled
    ouster_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "ouster_composable.launch.py")
        ),
        condition=IfCondition(launch_ouster),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_svo_path,
            declare_set_gravity_as_origin,
            declare_publish_tf,
            declare_publish_map_tf,
            declare_traversability_profile,
            declare_traversability,
            declare_zed,
            declare_ouster,
            declare_localization,
            zed_cmd,
            traversability_cmd,
            ouster_cmd,
            localization_cmd,
        ]
    )
