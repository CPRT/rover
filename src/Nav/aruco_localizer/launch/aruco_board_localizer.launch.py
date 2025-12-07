#!/usr/bin/env python3
"""
Launch file for ArUco Board Localizer Node

This node uses detected ArUco boards to provide robot localization estimates.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory("aruco_localizer")

    # Default path
    default_params_file = os.path.join(
        pkg_share, "config", "aruco_board_localizer_params.yaml"
    )

    # Declare launch arguments
    localizer_params_file_arg = DeclareLaunchArgument(
        "localizer_params_file",
        default_value=default_params_file,
        description="Path to the parameter file for the localizer node",
    )

    # Get launch configuration
    localizer_params_file = LaunchConfiguration("localizer_params_file")

    # ArUco Board Localizer Node
    localizer_node = Node(
        package="aruco_localizer",
        executable="aruco_board_localizer_node",
        name="aruco_board_localizer_node",
        output="screen",
        parameters=[localizer_params_file],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            localizer_params_file_arg,
            localizer_node,
        ]
    )
