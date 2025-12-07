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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
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

    return [local_ekf_cmd, global_ekf_cmd]


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

    return LaunchDescription(
        [
            use_sim_time_cmd,
            profile_cmd,
            OpaqueFunction(function=launch_setup),
        ]
    )
