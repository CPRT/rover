from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare("computer_vision")

    config_file = PathJoinSubstitution([pkg, "config", "keyboard_board_config.yaml"])
    calib_file = PathJoinSubstitution([pkg, "config", "camera_calibration.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/EndEffector/image_raw",
                description="Raw image topic from the arm camera",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="",
                description="CameraInfo topic (leave empty to use calibration file)",
            ),
            DeclareLaunchArgument(
                "output_frame",
                default_value="keyboard_center_link",
            ),
            DeclareLaunchArgument(
                "aruco_dict",
                default_value="DICT_4X4_50",
                description="ArUco dictionary name (e.g. DICT_4X4_50, DICT_4X4_100)",
            ),
            Node(
                package="computer_vision",
                executable="keyboard_pnp_locator_node",
                name="keyboard_pnp_locator_node",
                output="screen",
                parameters=[
                    {
                        "config_file": config_file,
                        "camera_calibration_file": calib_file,
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "output_frame": LaunchConfiguration("output_frame"),
                        "aruco_dict": LaunchConfiguration("aruco_dict"),
                    }
                ],
            ),
        ]
    )
