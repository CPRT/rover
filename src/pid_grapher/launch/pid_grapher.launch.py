import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="pid_grapher",
                executable="base_pid_grapher",
                name="base_pid_grapher_node",
            ),
        ]
    )
