#!/usr/bin/env python3
"""
Launch file for ArUco Board Localizer Node

This node uses detected ArUco boards to provide robot localization estimates.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("aruco_localizer"),
                "config",
                "aruco_board_localizer_params.yaml",
            ]
        ),
        description="Path to the parameter file for the localizer node",
    )

    # Get launch configuration
    params_file = LaunchConfiguration("params_file")

    # ArUco Board Localizer Node
    localizer_node = Node(
        package="aruco_localizer",
        executable="aruco_board_localizer_node",
        name="aruco_board_localizer_node",
        output="screen",
        parameters=[params_file],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            params_file_arg,
            localizer_node,
        ]
    )
