#!/usr/bin/env python3
"""
Launch file for ArUco Board Detector Node

This launch file starts the aruco_board_detector_node, loading parameters from
the config file. Only essential launch arguments are exposed for quick overrides.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory("aruco_localizer")

    # Default paths
    default_params_file = os.path.join(pkg_share, "config", "aruco_board_params.yaml")
    default_board_config_dir = os.path.join(pkg_share, "config", "aruco_boards")

    # Declare only essential launch arguments
    detector_params_file_arg = DeclareLaunchArgument(
        "detector_params_file",
        default_value=default_params_file,
        description="Path to ArUco board detector parameters YAML file",
    )

    board_config_dir_arg = DeclareLaunchArgument(
        "board_config_dir",
        default_value=default_board_config_dir,
        description="Directory containing board configuration YAML files",
    )

    # Create the node
    aruco_board_detector_node = Node(
        package="aruco_localizer",
        executable="aruco_board_detector_node",
        name="aruco_board_detector_node",
        output="screen",
        parameters=[
            LaunchConfiguration("detector_params_file"),
            {
                "board_config_dir": LaunchConfiguration("board_config_dir"),
            },
        ],
    )

    return LaunchDescription(
        [
            detector_params_file_arg,
            board_config_dir_arg,
            aruco_board_detector_node,
        ]
    )
