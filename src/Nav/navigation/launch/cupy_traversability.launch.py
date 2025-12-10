import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    package_name = "elevation_mapping_cupy"
    share_dir = get_package_share_directory(package_name)

    navigation_pkg = get_package_share_directory("navigation")
    core_param_path = os.path.join(
        navigation_pkg, "config", "cupy_elevation_mapping", "core_param.yaml"
    )
    spike_param_path = os.path.join(
        navigation_pkg, "config", "cupy_elevation_mapping", "spike_setup.yaml"
    )

    # Add verification
    if not os.path.exists(core_param_path):
        raise FileNotFoundError(f"Core param file not found: {core_param_path}")
    if not os.path.exists(spike_param_path):
        raise FileNotFoundError(f"Spike param file not found: {spike_param_path}")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    elevation_mapping_node_py = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            ParameterFile(core_param_path, allow_substs=True),
            spike_param_path,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([use_sim_time_arg, elevation_mapping_node_py])
