import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name="PhoenixContainerEEF",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="end_effector",
                parameters=[
                    {"id": 21},
                    {"max_voltage": 24.0},
                    {"max_current": 6.0},
                    {"brake_mode": True},
                ],
            ),
        ],
    )

    return launch.LaunchDescription(
        [
            container,
        ]
    )
