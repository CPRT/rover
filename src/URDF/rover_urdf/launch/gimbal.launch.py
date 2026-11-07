"""Standalone gimbal: mock hardware by default, or the rover USB Maestro."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        "gimbal_mock_hardware": "true",
        "gimbal_serial_port": "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00460938-if00",
        "gimbal_mount_xyz": "0 0 0",
        "gimbal_mount_rpy": "0 0 0",
        "mast_camera_xyz": "0 0 0",
        "mast_camera_rpy": "0 0 0",
    }
    xacro = PathJoinSubstitution(
        [FindPackageShare("rover_urdf"), "urdf", "rover_urdf.urdf.xacro"]
    )
    command = ["xacro ", xacro, " use_arm:=false use_drive:=false use_gimbal:=true"]
    for name in defaults:
        command.extend([" ", name, ":='", LaunchConfiguration(name), "'"])
    description = {
        "robot_description": ParameterValue(Command(command), value_type=str)
    }
    config = PathJoinSubstitution(
        [FindPackageShare("rover_urdf"), "config", "ros2_controllers.yaml"]
    )
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[description, config],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "gimbal_controller",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            ),
        ]
    )
