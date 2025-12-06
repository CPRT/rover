#!/usr/bin/env python3
"""
Launch file for Camera Publisher Node

This launch file starts the camera_publisher_node, loading parameters from
the config file. Only essential launch arguments are exposed for quick overrides.
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
    
    # Default paths
    default_params_file = os.path.join(pkg_share, "config", "camera_params.yaml")
    default_calibration_file = os.path.join(pkg_share, "config", "camera_intrinsics", "ErikKlarityCam.yaml")

    # Declare only essential launch arguments
    params_file_arg = DeclareLaunchArgument(
        "cam_params",
        default_value=default_params_file,
        description="Path to camera parameters YAML file",
    )

    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=default_calibration_file,
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
            LaunchConfiguration("cam_params"),
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
