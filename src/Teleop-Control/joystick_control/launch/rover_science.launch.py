from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_joystick_control = get_package_share_directory("joystick_control")
    parameters_file = os.path.join(pkg_joystick_control, "pxn.yaml")

    return LaunchDescription(
        [
            Node(
                package="ros_roboclaw",
                executable="roboclaw_node",
                name="roboclaw_elevator_node",
                namespace="elevator",
                parameters=[parameters_file],
                remappings=[("robo_duty_cycle", "/elevator/set")],
            ),
            Node(
                package="ros_roboclaw",
                executable="roboclaw_node",
                name="roboclaw_drill_node",
                namespace="drill",
                parameters=[parameters_file],
                remappings=[("robo_duty_cycle", "/drill/set")],
            ),
        ]
    )
