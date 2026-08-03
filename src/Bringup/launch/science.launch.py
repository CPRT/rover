import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    joystick_control_dir = get_package_share_directory("joystick_control")
    joy_parameters_file = os.path.join(joystick_control_dir, "3dpro.yaml")

    science_drill_control = Node(
        package="joystick_control",
        executable="drill",
        name="drill_teleop_node",
        parameters=[joy_parameters_file],
        remappings=[("/joy", "/arm/joy")],
    )
    talon_container = ComposableNodeContainer(
        name="PhoenixContainerScienceTeleop",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can1"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="elevator",
                parameters=[
                    {"interface": "can1"},
                    {"id": 20},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"watchdog_ms": 500},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="drill",
                parameters=[
                    {"interface": "can1"},
                    {"id": 21},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"watchdog_ms": 500},
                ],
            ),
        ],
        output="screen",
    )

    science_sensors = Node(
        package="science_sensors",
        executable="science_sensors",
        name="science_sensors",
        parameters=[{"interface": "can1"}],
    )

    panoramic = Node(
        package="science_python",
        executable="panoramic",
        name="panoramic",
    )

    ld = LaunchDescription()
    ld.add_action(science_drill_control)
    ld.add_action(talon_container)
    ld.add_action(science_sensors)
    ld.add_action(panoramic)
    return ld
