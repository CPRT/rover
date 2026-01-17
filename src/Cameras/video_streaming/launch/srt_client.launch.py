from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    srt_client_node = ComposableNode(
        package="video_streaming",
        plugin="video_streaming::SrtClientNode",
        name="srt_client_node",
        namespace="",
        parameters=[
            {
                "srt_uri": "srt://:7001?mode=listener"
            }
        ],
    )

    # Container_mt
    container = ComposableNodeContainer(
        name="test_client_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            srt_client_node
        ],
        output="screen",
    )

    return LaunchDescription([container])