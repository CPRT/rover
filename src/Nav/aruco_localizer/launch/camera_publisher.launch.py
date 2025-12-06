#!/usr/bin/env python3
"""
Launch file for Camera Publisher Node

This launch file starts the camera_publisher_node, loading parameters from
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
        default_value=PathJoinSubstitution([pkg_share, "config", "camera_params.yaml"]),
        description="Path to camera parameters YAML file",
    )

    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "camera_intrinsics", "ErikKlarityCam.yaml"]
        ),
        description="Path to camera calibration YAML file",
    )

    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="",
        description="Camera device path (empty = use params file default)",
    )

    # Create the node
    camera_publisher_node = Node(
        package="aruco_localizer",
        executable="camera_publisher_node",
        name="camera_publisher_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
            },
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            calibration_file_arg,
            camera_device_arg,
            camera_publisher_node,
        ]
    )
