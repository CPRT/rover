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
    generate_rsp_launch,
    generate_move_group_launch,
    generate_spawn_controllers_launch,
)


def load_yaml(package_name: str, file_path: str):
    """Load a YAML file from a package's share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def arm_launch(moveit_config, launch_package_path=None) -> LaunchDescription:
    """
    Launch a self-contained MoveIt demo.

    Includes:
      * robot_state_publisher
      * move_group
      * moveit_rviz
      * ros2_control_node + controller spawners
    """
    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareBooleanLaunchArg("use_composition", default_value=True))
    ld.add_action(
        DeclareBooleanLaunchArg("use_intra_process_comms", default_value=True)
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=False))

    use_composition = LaunchConfiguration("use_composition")
    use_intra_process_comms = LaunchConfiguration("use_intra_process_comms")

    # Robot state publisher
    ld.add_action(generate_rsp_launch(moveit_config))

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
    servo_yaml = load_yaml("arm_srdf", "config/arm_config.yaml")
    parameters = [
        {
            "moveit_servo": servo_yaml,
            "use_intra_process_comms": use_intra_process_comms,
        },
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        str(moveit_config.package_path / "config/ros2_controllers.yaml"),
    ]

    # Non-composed nodes
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                respawn_delay=2.0,
                parameters=parameters,
                remappings=[
                    ("/controller_manager/robot_description", "/robot_description")
                ],
            ),
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                parameters=parameters,
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher::robot_state_publisher",
                name="robot_state_publisher",
                parameters=parameters,
            ),
        ],
    )

    # Composable container
    container_name = "arm_container"
    container = ComposableNodeContainer(
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        name=container_name,
        output="screen",
        condition=IfCondition(use_composition),
    )

    # Composable nodes (loaded into container when composition is enabled)
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="controller_manager",
                plugin="controller_manager::ControllerManager",
                name="controller_server",
                parameters=parameters,
                remappings=[
                    ("/controller_manager/robot_description", "/robot_description")
                ],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_server",
                parameters=parameters,
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=parameters,
            ),
        ],
    )

    # Add actions to launch description
    ld.add_action(load_nodes)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    return ld


def generate_launch_description() -> LaunchDescription:
    moveit_config = (
        MoveItConfigsBuilder("arm_urdf", package_name="arm_srdf")
        .robot_description(file_path="config/arm_urdf.urdf.xacro")
        .to_moveit_configs()
    )
    return arm_launch(moveit_config)
