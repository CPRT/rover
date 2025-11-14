import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get package directories
    pkg_navigation = get_package_share_directory("navigation")
    pkg_localization = get_package_share_directory("localization")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # Declare launch arguments
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

    # Create a composable container for SLAM components
    slam_container = ComposableNodeContainer(
        name="slam_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Pointcloud to LaserScan conversion
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name="pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", "/ouster/points"),
                    ("scan", "/2d_ouster_scan"),
                ],
                parameters=[
                    {
                        "target_frame": "lidar_link",
                        "transform_tolerance": 0.01,
                        "min_height": -0.3,  # ouster is 0.425m above ground
                        "max_height": 0.5,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.01227184630308513,  # ouster in 512x10 mode so use 2*pi/512
                        "scan_time": 0.1,  # matches 10 Hz
                        "range_min": 1.5,  # avoids accidentally picking up the rover itself
                        "range_max": 20.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            # SLAM Toolbox
            ComposableNode(
                package="slam_toolbox",
                plugin="slam_toolbox::OnlineAsyncSlamToolbox",
                name="slam_toolbox",
                parameters=[
                    slam_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "publish_tf": False,  # False: Disable slam_toolbox publishing odom and map tf frames
                    },
                ],
                remappings=[
                    ("scan", "/2d_ouster_scan"),
                    ("pose", "/2dslam_pose"),
                ],
            ),
        ],
        output="screen",
    )

    # Launch Ouster with the SLAM container as target
    ouster_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "ouster_composable.launch.py")
        ),
        launch_arguments={
            "target_container": "/slam_container",
            "independent_container": "False",
        }.items(),
    )

    # Launch SLAM launch file with localization disabled
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
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
            "ekf_profile": "lidar_2dslam",
            "launch_gps": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam_params_file,
            slam_container,
            ouster_cmd,
            slam_cmd,
            localization_cmd,
        ]
    )
