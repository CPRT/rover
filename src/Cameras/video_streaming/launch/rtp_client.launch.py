from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    rtp_client_node = ComposableNode(
        package="video_streaming",
        plugin="video_streaming::RtpClientNode",
        name="rtp_client_node",
        namespace="",
        parameters=[{"port": 5004, "latency_ms": 40}],
    )

    # Container_mt
    container = ComposableNodeContainer(
        name="video_client_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[rtp_client_node],
        output="screen",
    )

    return LaunchDescription([container])
