import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- Args ---
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    control_yaml = LaunchConfiguration("control_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_description_pkg = DeclareLaunchArgument(
        "description_pkg",
        default_value="rover_urdf",
        description="Package that contains the robot xacro",
    )

    declare_xacro_file = DeclareLaunchArgument(
        "xacro_file",
        default_value="urdf/rover_urdf.urdf.xacro",
        description="Path to xacro file relative to description_pkg",
    )

    declare_control_yaml = DeclareLaunchArgument(
        "control_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rover_urdf"), "config", "ros2_controllers.yaml"]
        ),
        description="Chassis controller_manager params YAML",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    # --- controller_manager (ros2_control_node) ---
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            control_yaml,
        ],
        remappings=[
            ("controller_manager/robot_description", "/robot_description"),
            ("~/cmd_vel", "/cmd_vel"),
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
        parameters=[{"use_sim_time": use_sim_time}, control_yaml],
        output="screen",
    )

    chassis_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "chassis_controller",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    delayed_spawners = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[jsb_spawner, chassis_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            declare_description_pkg,
            declare_xacro_file,
            declare_control_yaml,
            declare_use_sim_time,
            controller_manager,
            delayed_spawners,
        ]
    )
