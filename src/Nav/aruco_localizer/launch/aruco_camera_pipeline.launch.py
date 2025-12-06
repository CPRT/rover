#!/usr/bin/env python3
"""
Combined Launch File for Camera Publisher and ArUco Board Detector

This launch file starts both the camera_publisher_node and the 
aruco_board_detector_node together for complete ArUco board detection pipeline.
Uses config files for most parameters, exposing only essential overrides.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare("aruco_localizer")

    # Declare only essential launch arguments
    camera_params_file_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "camera_params.yaml"]),
        description="Path to camera parameters YAML file",
    )

    aruco_params_file_arg = DeclareLaunchArgument(
        "aruco_params_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "aruco_board_params.yaml"]
        ),
        description="Path to ArUco board detector parameters YAML file",
    )

    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "camera_intrinsics", "ErikKlarityCam.yaml"]
        ),
        description="Path to camera calibration YAML file",
    )

    board_config_dir_arg = DeclareLaunchArgument(
        "board_config_dir",
        default_value=PathJoinSubstitution([pkg_share, "config", "aruco_boards"]),
        description="Directory containing board configuration YAML files",
    )

    # Include camera publisher launch file
    camera_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, "launch", "camera_publisher.launch.py"])]
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("camera_params_file"),
            "calibration_file": LaunchConfiguration("calibration_file"),
        }.items(),
    )

    # Include aruco board detector launch file
    aruco_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [pkg_share, "launch", "aruco_board_detector.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("aruco_params_file"),
            "board_config_dir": LaunchConfiguration("board_config_dir"),
        }.items(),
    )

    return LaunchDescription(
        [
            camera_params_file_arg,
            aruco_params_file_arg,
            calibration_file_arg,
            board_config_dir_arg,
            camera_publisher_launch,
            aruco_detector_launch,
        ]
    )
