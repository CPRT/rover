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
        parameters=[{"bottle_config": config_dir + "/bottle/bottle.txt"}],
    )

    streaming_node = ComposableNode(
        package="video_streaming",
        plugin="WebRtcNode",
        name="streaming_node",
        namespace="",
        parameters=[],
    )

    srt_node = ComposableNode(
        package="video_streaming",
        plugin="video_streaming::SrtNode",
        name="srt_node",
        namespace="",
        parameters=[
            {
                "srt_uri": "srt://:7001",
                "latency": 100,
                "iframe_interval": 30,
            }
        ],
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
            # streaming_node,
            srt_node,
        ],
        output="screen",
    )

    return LaunchDescription([container])
