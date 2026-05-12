import launch
import launch_ros.actions

PI_IP = "127.0.0.1"


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="mavros",
                executable="mavros_node",
                name="mavros_node",
                parameters=[
                    {"fcu_url": f"udp://127.0.0.1:14550@{PI_IP}:14550"},
                    {"fcu_protocol": "v2.0"},
                    {"tgt_system": 1},
                    {"tgt_component": 1},
                    {"plugin_allowlist": ["global_position"]},
                    {"respawn_mavros": False},
                ],
            )
        ]
    )
