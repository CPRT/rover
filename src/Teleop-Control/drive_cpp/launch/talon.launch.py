import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import subprocess
import os


def generate_launch_description():
    """Generate launch description with multiple components."""

    common_config_path = os.path.join(
        get_package_share_directory("drive_cpp"), "config", "talon_drive_common.yaml"
    )
    unique_config_path = os.path.join(
        get_package_share_directory("drive_cpp"), "config", "talon_drive_unique.yaml"
    )
    config_paths_lst = [common_config_path, unique_config_path]

    container_name = LaunchConfiguration("container_name")
    container_arg = DeclareLaunchArgument(
        "container_name",
        default_value="PhoenixContainer",
        description="Name of the target container",
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontLeft",
                parameters=config_paths_lst,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backLeft",
                parameters=config_paths_lst,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontRight",
                parameters=config_paths_lst,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backRight",
                parameters=config_paths_lst,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="drive_cpp",
                plugin="TalonDriveController",
                name="talon_drive_controller",
                parameters=config_paths_lst,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return launch.LaunchDescription(
        [
            container_arg,
            container,
        ]
    )
