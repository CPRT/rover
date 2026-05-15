# Copyright (C) 2023  Miguel Ángel González Santamaría

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from nav2_common.launch import RewrittenYaml

EKF_PROFILES = {
    "gps": {
        "local": "gps_ekf_local.yaml",
        "global": "gps_ekf_global.yaml",
    },
    "indoor_aruco_boards": {
        "local": "indoor_aruco_boards_ekf_local.yaml",
        "global": "indoor_aruco_boards_ekf_global.yaml",
    },
}
DEFAULT_PROFILE = "gps"


def launch_setup(context):
    # Retrieve launch configuration values
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    profile = LaunchConfiguration("ekf_profile").perform(context)

    # TF Telemetry configurations
    tf_publish_rate = LaunchConfiguration("tf_publish_rate").perform(context)
    # Note: allowed_frames is passed as a string representation of a list from the argument
    tf_allowed_frames = LaunchConfiguration("tf_allowed_frames").perform(context)

    # Set default profile if 'default' is selected
    if profile in ("default", ""):
        profile = DEFAULT_PROFILE

    # Check if the selected profile exists
    if profile not in EKF_PROFILES:
        raise ValueError(
            f"Unknown EKF profile: '{profile}'. Available profiles are: {list(EKF_PROFILES.keys())}"
        )

    # Load the EKF parameters based on the profile
    local_params_file = os.path.join(
        get_package_share_directory("localization"),
        "config",
        "ekf",
        EKF_PROFILES[profile]["local"],
    )
    global_params_file = os.path.join(
        get_package_share_directory("localization"),
        "config",
        "ekf",
        EKF_PROFILES[profile]["global"],
    )
    print(f"Using EKF profile: {profile}")

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_local_params = RewrittenYaml(
        source_file=local_params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    configured_global_params = RewrittenYaml(
        source_file=global_params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    local_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf",
        output="log",
        parameters=[configured_local_params],
        remappings=[
            ("odometry/filtered", "odometry/filtered/local"),
            ("accel/filtered", "/accel/local"),
        ],
    )
    global_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf",
        output="log",
        parameters=[configured_global_params],
        remappings=[
            ("odometry/filtered", "odometry/filtered/global"),
            ("accel/filtered", "/accel/global"),
        ],
    )

    # Throttled TF for Basestation Telemetry
    tf_telemetry_cmd = Node(
        package="compressed_telemetry_cpp",
        executable="tf_telemetry_node",
        name="tf_telemetry",
        output="screen",
        parameters=[
            {
                "publish_rate": float(tf_publish_rate),
                "log_stats": True,
                # We use eval or similar logic if needed, but ROS 2 params
                # usually handle string-to-array conversion for yaml-like strings.
                # Strip brackets, spaces, AND single/double quotes
                "allowed_frames": [
                    f.strip(" '\"") for f in tf_allowed_frames.strip("[]").split(",")
                ],
            }
        ],
    )

    return [local_ekf_cmd, global_ekf_cmd, tf_telemetry_cmd]


def generate_launch_description():
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    profile_cmd = DeclareLaunchArgument(
        "ekf_profile",
        default_value=DEFAULT_PROFILE,
        description="Select the EKF profile to use",
    )

    tf_rate_cmd = DeclareLaunchArgument(
        "tf_publish_rate",
        default_value="5.0",
        description="Rate at which to republish TF to telemetry",
    )

    # Default frames needed to see the robot moving on a map in RViz
    default_frames = "['map', 'odom', 'base_link', 'base_footprint']"
    tf_frames_cmd = DeclareLaunchArgument(
        "tf_allowed_frames",
        default_value=default_frames,
        description="List of frame IDs to allow through the telemetry link",
    )

    return LaunchDescription(
        [
            use_sim_time_cmd,
            profile_cmd,
            tf_rate_cmd,
            tf_frames_cmd,
            OpaqueFunction(function=launch_setup),
        ]
    )
