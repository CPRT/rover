import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package directories
    pkg_navigation = get_package_share_directory("navigation")
    pkg_localization = get_package_share_directory("localization")
    pkg_aruco = get_package_share_directory("aruco_localizer")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    svo_path = LaunchConfiguration("svo_path")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time if true"
    )

    declare_svo_path = DeclareLaunchArgument(
        "svo_path", default_value="live", description="Path to an input SVO file."
    )

    # Launch SLAM launch file with localization disabled
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "svo_path": svo_path,
            "launch_localization": "false",
            "launch_ouster": "false",
        }.items(),
    )

    # Launch localization with lidar_2dslam EKF profile
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ekf_profile": "indoor_aruco_boards",
            "launch_gps": "false",
        }.items(),
    )

    # Launch camera publisher
    # camera_publisher_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_aruco, "launch", "camera_publisher.launch.py")
    #     ),
    # )

    # Launch ArUco board detector
    aruco_detector_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_aruco, "launch", "aruco_board_detector.launch.py")
        ),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_svo_path,
            slam_cmd,
            localization_cmd,
            aruco_detector_cmd,
        ]
    )
