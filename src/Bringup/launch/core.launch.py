import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def get_included_launch_descriptions(launch_files):
    included_launches = []
    for pkg, file in launch_files:
        pkg_share = FindPackageShare(pkg).find(pkg)
        file_path = os.path.join(pkg_share, "launch", file)

        if file.endswith(".py"):
            source = PythonLaunchDescriptionSource(file_path)
        elif file.endswith(".xml"):
            source = XMLLaunchDescriptionSource(file_path)
        else:
            raise ValueError(f"Unsupported launch file format: {file}")

        included_launches.append(IncludeLaunchDescription(source))

    return included_launches


def generate_launch_description():
    launch_files = [
        ("servo_pkg", "usb_servo_launch.launch.py"),
        ("gpio_controller", "core_gpio_devices.launch.py"),
    ]

    snmp_node = Node(
        package="system-telemetry-cpp",
        executable="snmp_network_stats",
        name="snmp_network_stats",
        output="screen",
    )

    status_node = Node(
        package="system-telemetry-cpp",
        executable="node_status_publisher",
        name="node_status_publisher",
    )

    system_telemetry_node = Node(
        package="system-telemetry-cpp",
        executable="system_telemetry_publisher",
        name="system_telemetry_node",
    )

    return LaunchDescription(
        get_included_launch_descriptions(launch_files) + [snmp_node, status_node]
    )
