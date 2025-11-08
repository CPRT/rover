import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("video_streaming"), "config")

    camera_params_file = os.path.join(config_dir, "cameras.yaml")
    # Define each composable node
    input_node = ComposableNode(
        package="video_streaming",
        plugin="InputNode",
        name="input_node",
        namespace="",
        parameters=[camera_params_file],
    )

    detect_node = ComposableNode(
        package="video_streaming",
        plugin="DetectNode",
        name="detect_node",
        namespace="",
        parameters=[],
    )

    streaming_node = ComposableNode(
        package="video_streaming",
        plugin="WebRtcNode",
        name="streaming_node",
        namespace="",
        parameters=[],
    )

    # Create a container for all 3 components
    container = ComposableNodeContainer(
        name="video_streaming_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            input_node,
            detect_node,
            streaming_node,
        ],
        output="screen",
    )

    return LaunchDescription([container])
