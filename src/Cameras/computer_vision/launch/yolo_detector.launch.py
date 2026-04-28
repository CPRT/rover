from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="mallet",
        description='Model to use: "mallet" or "bottle"',
    )

    output_arg = DeclareLaunchArgument(
        "output",
        default_value="/tmp/yolo_recording.mp4",
        description="Output video file path (use absolute path)",
    )

    confidence_arg = DeclareLaunchArgument(
        "confidence",
        default_value="0.25",
        description="Confidence threshold for detections (0.0-1.0)",
    )

    fps_arg = DeclareLaunchArgument(
        "fps", default_value="30.0", description="Output video frames per second"
    )

    use_compressed_arg = DeclareLaunchArgument(
        "use_compressed",
        default_value="false",
        description="Use compressed image topic (true) or uncompressed (false)",
    )

    labels_arg = DeclareLaunchArgument(
        "labels",
        default_value="",
        description="Label mapping in format 'old1:new1,old2:new2' (e.g., '0:Mallet,1:Bottle'). Leave empty to use model-specific defaults.",
    )

    # Map model name to file path
    model_path = PythonExpression(
        [
            "'",
            "/usr/local/zed/RANDOM/MalletYoloModel.pt",
            "' if '",
            LaunchConfiguration("model"),
            "' == 'mallet' else ('",
            "/usr/local/zed/RANDOM/BottleYoloModel.pt",
            "' if '",
            LaunchConfiguration("model"),
            "' == 'bottle' else '",
            LaunchConfiguration("model"),
            "')",
        ]
    )

    # Set label mapping based on model or custom labels
    label_map = PythonExpression(
        [
            "'",
            LaunchConfiguration("labels"),
            "' if '",
            LaunchConfiguration("labels"),
            "' else ('0:Mallet' if '",
            LaunchConfiguration("model"),
            "' == 'mallet' else ('0:Bottle' if '",
            LaunchConfiguration("model"),
            "' == 'bottle' else ''))",
        ]
    )

    # YOLO detector node
    yolo_detector_node = Node(
        package="computer_vision",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        parameters=[
            {
                "model_path": model_path,
                "confidence_threshold": LaunchConfiguration("confidence"),
                "video_output_path": LaunchConfiguration("output"),
                "video_fps": LaunchConfiguration("fps"),
                "use_compressed": LaunchConfiguration("use_compressed"),
                "label_map": label_map,
            }
        ],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            model_arg,
            output_arg,
            confidence_arg,
            fps_arg,
            use_compressed_arg,
            labels_arg,
            yolo_detector_node,
        ]
    )
