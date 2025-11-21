from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction
from launch.substitutions import PythonExpression
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def test_launch(moveit_config, launch_package_path=None):
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("use_composition", default_value=True))
    ld.add_action(
        DeclareBooleanLaunchArg("use_intra_process_comms", default_value=True)
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=False))
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    use_composition = LaunchConfiguration("use_composition")
    use_intra_process_comms = LaunchConfiguration("use_intra_process_comms")
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
    container_name = "arm_container"
    container = ComposableNodeContainer(
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        name=container_name,
        output="screen",
        condition=IfCondition(use_composition),
    )

    # Group of actions to load composable nodes when composition is enabled
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
                parameters=parameters
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=parameters,
            ),
        ],
    )

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(container)
    return ld


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm_urdf", package_name="arm_srdf")
        .robot_description(file_path="config/arm_urdf.urdf.xacro")
        .to_moveit_configs()
    )
    return test_launch(moveit_config)
