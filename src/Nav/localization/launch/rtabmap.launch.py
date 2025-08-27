import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.actions import LogInfo

profiles = {
    "zed_only": {
        "parameters": {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan_cloud": False,
            "approx_sync": True,
            "publish_tf": False,
            "qos_camera_info": 2,
            "topic_queue_size": 2,
            "sync_queue_size": 10,
            "wait_for_transform": 0.3,
            "Grid/DepthDecimation": "2",
            "Grid/RangeMin": "0.5",
            "Grid/RangeMax": "5.0",
            "Grid/MinClusterSize": "10",
            "Grid/MaxGroundAngle": "45",
            "Grid/NormalK": "10",
            "Grid/CellSize": "0.1",
            "Grid/FlatObstacleDetected": "false",
            "Grid/MapFrameProjection": "true",
            "GridGlobal/UpdateError": "0.01",
            "GridGlobal/MinSize": "1000",
            "GridGlobal/OccupancyThr": "0.5",
            "Reg/Strategy": "1",
        },
        "remappings": [
            ("rgb/image", "zed/zed_node/rgb/image_rect_color"),
            ("rgb/camera_info", "zed/zed_node/rgb/camera_info"),
            ("depth/image", "zed/zed_node/depth/depth_registered"),
            ("odom", "odometry/filtered/local"),
            ("imu", "zed/zed_node/imu/data"),
            ("map", "map"),
        ],
    },
    "lidar_only": {
        "parameters": {
            "frame_id": "base_link",
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_scan_cloud": True,
            "approx_sync": True,
            "publish_tf": False,
            "qos_camera_info": 2,
            "qos_imu": 2,
            "topic_queue_size": 2,
            "sync_queue_size": 10,
            "wait_for_transform": 0.3,
            "Grid/DepthDecimation": "2",
            "Grid/RangeMin": "1.5",
            "Grid/RangeMax": "25.0",
            "Grid/MinClusterSize": "4",
            "Grid/MaxGroundAngle": "35",
            "Grid/NormalK": "20",
            "Grid/CellSize": "0.1",
            "Grid/FlatObstacleDetected": "false",
            "GridGlobal/UpdateError": "0.01",
            "GridGlobal/MinSize": "200",
            "GridGlobal/OccupancyThr": "0.5",
            "Reg/Strategy": "1",
        },
        "remappings": [
            ("scan_cloud", "ouster/points"),
            # ("rgb/camera_info", "camera/camera_info"),
            # ("depth/image", "zed/depth_image"),
            # ("imu", "ouster/imu"),
            ("imu", "zed/zed_node/imu/data"),
            ("odom", "odometry/filtered/local"),
            ("goal", "goal_pose"),
            ("map", "map"),
        ],
    },
}

default_profile = "lidar_only"


def launch_setup(context):
    # This function is executed at launch time

    # Get resolved launch argument values
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz").perform(context)
    selected_profile = LaunchConfiguration("profile").perform(context)

    # Validate the selected profile
    if selected_profile not in profiles:
        raise ValueError(
            f"Unknown RTAB-Map profile: '{selected_profile}'. Available profiles are: {list(profiles.keys())}"
        )

    profile_data = profiles[selected_profile]

    # Path to the config directory in your custom package
    config_dir = os.path.join(get_package_share_directory("localization"), "config")

    # The rtabmap.yaml file typically contains parameters for rtabmap_viz itself,
    # or general parameters that overlay with the node-specific ones.
    params_file = os.path.join(config_dir, "rtabmap.yaml")

    # Combine profile-specific parameters with use_sim_time
    # Note: parameters is a list of dictionaries in ROS 2 launch,
    # so we wrap the profile_data["parameters"] in a list.
    parameters_for_node = [profile_data["parameters"]]

    # Add use_sim_time parameter to the main RTAB-Map node
    parameters_for_node[0]["use_sim_time"] = use_sim_time == "true"

    remappings_for_node = profile_data["remappings"]

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=parameters_for_node,
        remappings=remappings_for_node,
        arguments=[
            "-d",
            "--delete_db_on_start",
            "--ros-args",
            "--log-level",
            "Warn",
        ],
    )

    # RTAB-Map Viz Node (conditionally launched)
    # This node also needs the rtabmap.yaml parameters (if any) and use_sim_time.
    # It also needs the same remappings to display data correctly from rtabmap.
    rtabmap_viz_node = Node(
        condition=IfCondition(launch_rtabmapviz),
        package="rtabmap_viz",
        executable="rtabmap_viz",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time == "true"}],
        remappings=remappings_for_node,
    )

    return [
        LogInfo(msg=f"Launching RTAB-Map with profile: {selected_profile}"),
        rtabmap_node,
        rtabmap_viz_node,
    ]


def generate_launch_description():
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Whether to launch rtabmapviz",
    )

    profile_cmd = DeclareLaunchArgument(
        "profile",
        default_value=default_profile,
        description="Select the RTAB-Map profile to use (e.g., zed_only, lidar_only)",
    )

    return LaunchDescription(
        [
            use_sim_time_cmd,
            launch_rtabmapviz_cmd,
            profile_cmd,
            OpaqueFunction(function=launch_setup),
        ]
    )
