import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("video_streaming"), "config")

    camera_params_file = os.path.join(config_dir, "cameras.yaml")
    preset_params_file = os.path.join(config_dir, "presets.yaml")
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
        parameters=[
            {
                "bottle_config": config_dir + "/bottle/bottle.txt",
                "mallet_config": config_dir + "/mallet/mallet.txt",
                "rockpick_config": config_dir + "/hammer/hammer.txt",
                # Options: NONE, ARUCO, MORSE, WATER_BOTTLE, MALLET, ROCKPICK
                "detection_type": "NONE",
                # Morse LED params (used when detection_type == MORSE)
                "start_detection": False,
                "calibrate": True,
                "draw_roi": True,
            }
        ],
    )

    rtp_node = ComposableNode(
        package="video_streaming",
        plugin="video_streaming::RtpNode",
        name="rtp_node",
        namespace="",
        parameters=[{"dest_ip": "192.168.0.20", "dest_port": 5004, "bitrate": 500000}],
    )

    capture_node = ComposableNode(
        package="video_streaming",
        plugin="VideoCaptureNode",
        name="capture_node",
        namespace="",
        parameters=[camera_params_file],
    )

    # Streams the mosaic compositor output via WebRTC on a separate signalling
    # server (default port 8444) so the WebUI mosaic tile can connect to it.
    mosaic_webrtc_node = ComposableNode(
        package="video_streaming",
        plugin="video_streaming::MosaicWebRtcNode",
        name="mosaic_webrtc_node",
        namespace="",
        parameters=[{"signalling_port": 8444, "bitrate": 1000000, "framerate": 5}],
    )

    preset_node = ComposableNode(
        package="video_streaming",
        plugin="PresetNode",
        name="preset_node",
        namespace="",
        parameters=[preset_params_file],
    )

    container = ComposableNodeContainer(
        name="video_streaming_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            input_node,
            detect_node,
            capture_node,
            rtp_node,
            mosaic_webrtc_node,
            preset_node,
        ],
        output="screen",
    )

    return LaunchDescription([container])
