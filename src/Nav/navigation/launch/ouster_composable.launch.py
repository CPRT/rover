# Copyright 2023 Ouster, Inc.

"""Launch ouster nodes using a composite container."""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import os


def generate_launch_description():
    """
    Generate a launch description for running ouster_ros components either
    inside a specified target container or in an independent component container.
    """
    ouster_ros_pkg_dir = get_package_share_directory("ouster_ros")

    # Parameter and mask file paths
    params_file = os.path.join(
        get_package_share_directory("navigation"),
        "config",
        "ouster",
        "driver_params.yaml",
    )

    mask_file = os.path.join(
        get_package_share_directory("navigation"),
        "lidar_masks",
        "ouster_mask_combined_shifted.png",
    )

    # Launch arguments
    ouster_ns = LaunchConfiguration("ouster_ns")
    ouster_ns_arg = DeclareLaunchArgument("ouster_ns", default_value="ouster")

    rviz_enable = LaunchConfiguration("viz")
    rviz_enable_arg = DeclareLaunchArgument("viz", default_value="False")

    auto_start = LaunchConfiguration("auto_start")
    auto_start_arg = DeclareLaunchArgument("auto_start", default_value="True")

    independent_container = LaunchConfiguration("independent_container")
    independent_container_arg = DeclareLaunchArgument(
        "independent_container",
        default_value="False",
        description="If true, launch Ouster nodes in their own container.",
    )

    target_container = LaunchConfiguration("target_container")
    target_container_arg = DeclareLaunchArgument(
        "target_container",
        default_value="/zed/zed_container",
        description="Container name to load Ouster composable nodes into when not launched independently.",
    )

    # Define the Ouster driver as a composable node
    os_driver = ComposableNode(
        package="ouster_ros",
        plugin="ouster_ros::OusterDriver",
        name="os_driver",
        namespace=ouster_ns,
        parameters=[
            params_file,
            {
                "auto_start": auto_start,
                "mask_path": mask_file,
                "use_intra_process_comms": True,
            },
        ],
    )

    # Load the composable node into an existing container when not independent
    load_ouster_nodes_into_zed_container = LoadComposableNodes(
        composable_node_descriptions=[os_driver],
        target_container=target_container,
        condition=IfCondition(PythonExpression(["not ", independent_container])),
    )

    # Create an independent component container for Ouster when requested
    ouster_container = ComposableNodeContainer(
        name="ouster_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[os_driver],
        condition=IfCondition(independent_container),
        output="screen",
    )

    # Optional RViz launch inclusion
    rviz_launch_file_path = Path(ouster_ros_pkg_dir) / "launch" / "rviz.launch.py"
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(rviz_launch_file_path)]),
        condition=IfCondition(rviz_enable),
    )

    return launch.LaunchDescription(
        [
            ouster_ns_arg,
            rviz_enable_arg,
            auto_start_arg,
            independent_container_arg,
            target_container_arg,
            rviz_launch,
            load_ouster_nodes_into_zed_container,
            ouster_container,
        ]
    )
