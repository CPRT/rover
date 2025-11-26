import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory("navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time if true"
    )

    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            pkg_navigation, "config", "slam_toolbox", "slam_toolbox_params.yaml"
        ),
        description="Path to slam_toolbox parameters file",
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "publish_tf": True,
            },
        ],
        remappings=[
            ("scan", "/ouster_2d_scan"),
            ("pose", "/slam_2d_pose"),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam_params_file,
            slam_toolbox_node,
        ]
    )
