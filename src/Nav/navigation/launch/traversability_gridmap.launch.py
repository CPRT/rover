import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, OpaqueFunction

profiles = {
    "zed_only": {
        "config_files": [
            "spike_robot.yaml",
            "zed2i_elevation_mapping.yaml",
            "zed2i_sensor_processor.yaml",
            "postprocessing_traversability.yaml",
        ],
        "input_sources": [
            "zed2i_pointcloud",
        ],
    },
    "lidar_only": {
        "config_files": [
            "spike_robot.yaml",
            "ouster_elevation_mapping.yaml",
            "ouster_OS0_32U_sensor_processor.yaml",
            "postprocessing_traversability.yaml",
        ],
        "input_sources": [
            "ouster_OS0_32U_pointcloud",
        ],
    },
    "all": {
        "config_files": [
            "spike_robot.yaml",
            "ouster_elevation_mapping.yaml",
            "postprocessing_traversability.yaml",
        ],
        "input_sources": ["zed2i_pointcloud", "ouster_OS0_32U_pointcloud"],
    },
}

default_profile = "all"


def launch_setup(context):
    # This function is executed at launch time, after arguments are resolved

    # Get the selected profile from the launch argument
    selected_profile = launch.substitutions.LaunchConfiguration("profile").perform(
        context
    )

    # Validate the selected profile
    if selected_profile not in profiles:
        raise ValueError(
            f"Unknown profile: '{selected_profile}'. Available profiles are: {list(profiles.keys())}"
        )

    # Use the config_files list from the selected profile
    list_files = profiles[selected_profile]["config_files"]
    input_sources = profiles[selected_profile]["input_sources"]

    navigation_package_dir = get_package_share_directory("navigation")
    config_dir = os.path.join(navigation_package_dir, "config", "elevation_mapping")
    list_params = []

    for file in list_files:
        file_path = os.path.join(config_dir, file)
        if not os.path.isfile(file_path):
            raise FileNotFoundError(
                f"Config file not found for profile '{selected_profile}': {file_path}"
            )
        list_params.append(file_path)

    # Add the input sources to the parameters to pick which pointcloud topics to use
    list_params.append({"inputs": input_sources})

    node = ComposableNode(
        package="elevation_mapping",
        plugin="elevation_mapping::ElevationMapNode",
        name="elevation_mapping",
        parameters=list_params,
        extra_arguments=[{"use_intra_process_comms": True}, {"inputs": input_sources}],
    )

    return [
        launch.actions.LogInfo(
            msg=f"Loading elevation mapping with profile: {selected_profile}"
        ),
        launch.actions.LogInfo(
            msg=f"Waiting to compose gridmap node into zed container"
        ),
        LoadComposableNodes(
            composable_node_descriptions=[node],
            target_container="/zed/zed_container",
        ),
    ]


def generate_launch_description():
    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value=default_profile,
        description="Select the elevation mapping profile to use (e.g., zed_only, lidar_only)",
    )

    return launch.LaunchDescription(
        [
            profile_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
