from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("compressed_telemetry_cpp"), "rviz", "compressed_nav2.rviz"]
    )

    costmap_decompressor = Node(
        package="compressed_telemetry_cpp",
        executable="costmap_decompressor_node",
        name="costmap_decompressor",
        output="screen",
    )

    gridmap_decompressor = Node(
        package="compressed_telemetry_cpp",
        executable="gridmap_decompressor_node",
        name="gridmap_decompressor",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        costmap_decompressor,
        gridmap_decompressor,
        rviz,
    ])
