"""
Launch file for the arm control package. Starts everything outside of ros2_control needed to control the arm, including move_group, moveit_rviz, and the servo node.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, ComposableNodeContainer

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch_ros.descriptions import ComposableNode


def load_yaml(package_name: str, file_path: str):
    """Load a YAML file from a package's share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def arm_launch(moveit_config, launch_package_path=None) -> LaunchDescription:
    """
    Launch a self-contained MoveIt demo.

    Includes:
      * move_group
      * moveit_rviz (Optional)
      * ros2_control_node + controller spawners
    """
    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=False))

    # -------------------------------------------------------------------------
    # move_group: DO NOT pass robot_description as a parameter.
    # This allows MoveIt to fall back to subscribing to ~/robot_description.
    # We remap that private topic to the global /robot_description published by
    # robot_state_publisher (most common setup).
    # -------------------------------------------------------------------------
    move_group_params = [
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.pilz_cartesian_limits,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        moveit_config.trajectory_execution,
        moveit_config.planning_scene_monitor,
        {"publish_robot_description_semantic": True},
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    ld.add_action(move_group)

    ld.add_action(
        GroupAction(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            actions=[generate_moveit_rviz_launch(moveit_config)],
        )
    )

    servo_yaml = load_yaml("arm_control", "config/arm_config.yaml")
    servo_parameters = [
        {
            "moveit_servo": servo_yaml,
            "publish_frequency": 100.0,
            "butterworth_filter_coeff": 2.0,
        },
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
    ]

    servo_component = ComposableNode(
        package="moveit_servo",
        plugin="moveit_servo::ServoNode",
        name="servo_node",
        parameters=servo_parameters,
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    eef_component = ComposableNode(
        package="ros_phoenix",
        plugin="ros_phoenix::TalonSRX",
        name="end_effector",
        parameters=[
            {"id": 15},
            {"max_voltage": 24.0},
            {"max_current": 6.0},
            {"brake_mode": True},
        ],
    )
    joystick_control_dir = get_package_share_directory("joystick_control")
    joy_parameters_file = os.path.join(joystick_control_dir, "pxn.yaml")

    arm_control_component = ComposableNode(
        package="joystick_control",
        plugin="joystick_control::ArmTeleop",
        name="arm_teleop_node",
        parameters=[joy_parameters_file],
        remappings=[("/joy", "/arm/joy")],
    )

    move_group_interface_component = ComposableNode(
        package="arm_control",
        plugin="arm_control::MoveGroupNode",
        name="move_group_interface",
        parameters=[{"vel_scaling": 0.5, "acc_scaling": 0.5}],
    )

    container = ComposableNodeContainer(
        name="arm_control_container",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            eef_component,
            servo_component,
            arm_control_component,
            move_group_interface_component,
        ],
    )

    ld.add_action(container)

    return ld


def generate_launch_description() -> LaunchDescription:
    moveit_config = MoveItConfigsBuilder(
        "rover_urdf", package_name="arm_control"
    ).to_moveit_configs()
    return arm_launch(moveit_config)
