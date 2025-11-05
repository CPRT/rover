import launch
import launch_ros.actions
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from math import pi
from launch.substitutions import LaunchConfiguration


PUBLISH_PERIOD = 1


def generate_launch_description():
    """Generate launch description with multiple components."""
    # base_input_type = LaunchConfiguration("base_input_type", default=1) + 1
    base_input_type = 2

    container = ComposableNodeContainer(
        name="PhoenixContainerArm",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="base",
                parameters=[
                    {"id": 10},
                    {"P": 40.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"sensor_multiplier": (2 * pi / 4096)},
                    {"sensor_offset": 0.0},
                    {"input_type": base_input_type},
                    {"max_voltage": 6.0},
                    {"period_ms": PUBLISH_PERIOD},
                    {"non_continuous": True},
                    {"watchdog_ms": 300},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="act1",
                parameters=[
                    {"id": 11},
                    {"P": 70.0},
                    {"I": 0.0},
                    {"D": 0.5},
                    {"input_type": base_input_type},
                    {"invert_sensor": True},
                    {"sensor_multiplier": (2 * pi / 4096)},
                    {"sensor_offset": -3.14},
                    {"max_voltage": 22.0},
                    {"period_ms": PUBLISH_PERIOD},
                    {"watchdog_ms": 300},
                    {"non_continuous": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="act2",
                parameters=[
                    {"id": 14},
                    {"P": 10.0},
                    {"I": 0.01},
                    {"D": 3.0},
                    {"input_type": base_input_type},
                    {"invert": True},
                    {"sensor_multiplier": (2 * pi / 4096)},
                    {"sensor_offset": -2.02401481},
                    {"max_voltage": 22.0},
                    {"period_ms": PUBLISH_PERIOD},
                    {"watchdog_ms": 300},
                    {"non_continuous": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="elbow",
                parameters=[
                    {"id": 15},
                    {"P": 4.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"max_voltage": 22.0},
                    {"sensor_multiplier": (2 * pi / 4096)},
                    {"sensor_offset": 0.826862284},
                    {"input_type": base_input_type},
                    {"watchdog_ms": 300},
                    {"period_ms": PUBLISH_PERIOD},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="wristTilt",
                parameters=[
                    {"id": 12},
                    {"P": 20.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"max_voltage": 22.0},
                    {"sensor_multiplier": (2 * pi / 4096)},
                    {"sensor_offset": 4.0},
                    {"input_type": base_input_type},
                    {"invert_sensor": True},
                    {"invert": True},
                    {"period_ms": PUBLISH_PERIOD},
                    {"watchdog_ms": 300},
                    {"non_continuous": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="wristTurn",
                parameters=[
                    {"id": 13},
                    {"P": 0.008},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"invert": True},
                    {"invert_sensor": True},
                    {
                        "sensor_multiplier": 0.00000315420949
                    },  # 2pi/(WRISTTURN_GEARBOX * WRISTTURN_GEAR)
                    {"period_ms": PUBLISH_PERIOD},
                    {"watchdog_ms": 300},
                    {"max_voltage": 22.0},
                    {"input_type": base_input_type},
                ],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [LogInfo(msg=["Running with control mode: 2"]), container]
    )
