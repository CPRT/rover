import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="science_sensors",
                executable="gas_sensor",
                name="gas_sensor",
                parameters=[
                    {"sea_level_pressure_hpa": 1013.25},
                ],
            ),
        ]
    )
