import launch
import launch_ros.actions

PI_IP = "192.168.0.25"


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="mavros",
                executable="mavros_node",
                name="mavros_node",
                parameters=[
                    {"fcu_url": f"udp://@{PI_IP}:14550"},
                    {"fcu_protocol": "v2.0"},
                    {"tgt_system": 1},
                    {"tgt_component": 1},
                    {"plugin_allowlist": ["global_position"]},
                    {"respawn_mavros": False},
                ],
            )
        ]
    )
