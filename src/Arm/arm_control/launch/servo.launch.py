import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes,
)
from launch_ros.descriptions import ComposableNode

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_spawn_controllers_launch,
    generate_moveit_rviz_launch,
)


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

    # Move group
    ld.add_action(generate_move_group_launch(moveit_config))

    # Controllers
    ld.add_action(generate_spawn_controllers_launch(moveit_config))

    # RViz (optional)
    ld.add_action(
        GroupAction(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            actions=[generate_moveit_rviz_launch(moveit_config)],
        )
    )

    # Common parameters
    servo_yaml = load_yaml("arm_control", "config/arm_config.yaml")
    parameters = [
        {
            "moveit_servo": servo_yaml,
            "publish_frequency": 15.0,
        },
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        str(launch_package_path / "config/ros2_controllers.yaml"),
    ]
    # Control Node does not implement a component option
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=parameters,
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=parameters,
        output="screen",
    )

    # Add actions to launch description
    ld.add_action(control_node)
    ld.add_action(servo_node)
    return ld


def generate_launch_description() -> LaunchDescription:
    urdf_pkg = get_package_share_directory("rover_urdf")
    urdf_path = os.path.join(
        urdf_pkg,
        "urdf",
        "rover_urdf.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("rover_urdf", package_name="arm_control")
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )
    return arm_launch(moveit_config)
