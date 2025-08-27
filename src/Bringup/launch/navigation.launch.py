from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
import os


def get_included_launch_descriptions(launch_files):
    included_launches = []
    for pkg, file in launch_files:
        pkg_share = FindPackageShare(pkg).find(pkg)
        file_path = os.path.join(pkg_share, "launch", file)
        included_launches.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(file_path))
        )
    return included_launches


def generate_launch_description():
    launch_files = [
        ("navigation", "nav2.launch.py"),
        ("navigation", "rtabmap.launch.py"),
        ("nav_commanders", "gps_commander.launch.py"),
    ]
    lights_node = Node(
        package="gpio_controller",
        executable="lights",
        name="lights_controller_node",  # Assign a unique name to the node
        output="screen",  # Show output in the console
        emulate_tty=True,  # Required for output to show in terminal when using 'screen'
        # parameters=[{'some_param': 'some_value'}] # Uncomment and add parameters if needed
    )
    return LaunchDescription(
        get_included_launch_descriptions(launch_files) + [lights_node]
    )
