from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1️⃣ Include talon.launch.py
    talon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arm_srdf"), "launch", "talon.launch.py"
            )
        )
    )

    # 2️⃣ Launch MStates node
    mstates_node = Node(
        package="Mstates",
        executable="MStates",  # make sure this matches the executable name
        name="MStates_node",
        output="screen",
    )

    # 3️⃣ Launch pid_controll.py node
    pid_controll_node = Node(
        package="pid_controll",
        executable="pid_controll",  # this is the console_script entry or script name
        name="pid_controll_node",
        output="screen",
    )

    # 4️⃣ Launch joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "dev": "/dev/input/js0",  # path to joystick device
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }
        ],
    )

    return LaunchDescription([talon_launch, mstates_node, joy_node, pid_controll_node])
