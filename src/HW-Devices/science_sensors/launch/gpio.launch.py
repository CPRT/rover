import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="science_sensors",
                executable="pi_gpio_controller",
                name="microscope_light",
                parameters=[
                    {"service_name": "/microscope_light"},
                    {"gpio_pins": [11, 6]},
                ],
            ),
            launch_ros.actions.Node(
                package="science_sensors",
                executable="pi_gpio_controller",
                name="raman_light",
                parameters=[
                    {"service_name": "/raman_light"},
                    {"gpio_pins": [7]},
                ],
            ),
            launch_ros.actions.Node(
                package="science_sensors",
                executable="pi_gpio_reader",
                name="gpio_reader_node",
                parameters=[
                    {"gpio_pins": [12]},
                    {"interval": 1.0},
                ],
                remappings=[
                    ("/gpio/12", "/science/ground"),
                ],
            ),
        ]
    )
