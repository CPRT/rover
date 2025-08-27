import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import subprocess


def generate_launch_description():
    """Generate launch description with multiple components."""
    try:
        subprocess.run(["sudo", "enablecan.sh"], check=True)
    except:
        print("enablecan.sh not found")
    container = ComposableNodeContainer(
        name="PhoenixContainerScience",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="platform",
                parameters=[
                    {"id": 7},
                    {"P": 2.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="drill",
                parameters=[
                    {"id": 8},
                    {"max_voltage": 24.0},
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
