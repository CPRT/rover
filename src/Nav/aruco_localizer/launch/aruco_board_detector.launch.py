#!/usr/bin/env python3
"""
Launch file for ArUco Board Detector Node

This launch file starts the aruco_board_detector_node, loading parameters from
the config file. Only essential launch arguments are exposed for quick overrides.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare("aruco_localizer")

    # Declare only essential launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "aruco_board_params.yaml"]
        ),
        description="Path to ArUco board detector parameters YAML file",
    )

    board_config_dir_arg = DeclareLaunchArgument(
        "board_config_dir",
        default_value=PathJoinSubstitution([pkg_share, "config", "aruco_boards"]),
        description="Directory containing board configuration YAML files",
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="",
        description="Image topic to subscribe to (empty = use params file default)",
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="",
        description="Camera info topic to subscribe to (empty = use params file default)",
    )

    # Create the node
    aruco_board_detector_node = Node(
        package="aruco_localizer",
        executable="aruco_board_detector_node",
        name="aruco_board_detector_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "board_config_dir": LaunchConfiguration("board_config_dir"),
            },
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            board_config_dir_arg,
            image_topic_arg,
            camera_info_topic_arg,
            aruco_board_detector_node,
        ]
    )
