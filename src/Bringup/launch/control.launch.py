import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():
    desc_dir = get_package_share_directory("rover_urdf")
    urdf_file = os.path.join(desc_dir, "urdf", "rover_urdf.urdf.xacro")
    control_yaml = os.path.join(desc_dir, "config", "ros2_controllers.yaml")
    use_arm = LaunchConfiguration("use_arm")
    use_drive = LaunchConfiguration("use_drive")
    joystick_control_dir = get_package_share_directory("joystick_control")
    joy_parameters_file = os.path.join(joystick_control_dir, "pxn.yaml")

    declare_use_arm_cmd = DeclareLaunchArgument(
        "use_arm",
        default_value="true",
        description="Enable the arm",
    )

    declare_use_drive_cmd = DeclareLaunchArgument(
        "use_drive",
        default_value="true",
        description="Enable the drive base",
    )

    robot_description = ParameterValue(
        Command(
            [
                "ros2 run xacro xacro ",
                urdf_file,
                " use_arm:=",
                use_arm,
                " use_drive:=",
                use_drive,
            ]
        ),
        value_type=str,
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[control_yaml],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("~/cmd_vel", "/cmd_vel"),
        ],
    )
    delayed_controller_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=start_robot_state_publisher_cmd,
            on_start=[
                controller_manager,
            ],
        )
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120",
        ],
        parameters=[control_yaml],
        output="screen",
    )
    chassis_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "chassis_controller",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120",
        ],
        parameters=[control_yaml],
        output="screen",
        condition=IfCondition(use_drive),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120",
        ],
        parameters=[control_yaml],
        output="screen",
        condition=IfCondition(use_arm),
    )

    delayed_spawners = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                jsb_spawner,
                chassis_controller_spawner,
                arm_controller_spawner,
            ],
        )
    )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arm_control"),
                "launch",
                "arm.launch.py",
            )
        ),
        condition=IfCondition(use_arm),
    )

    drive_control_node = Node(
        package="joystick_control",
        executable="drive",
        name="drive_teleop_node",
        parameters=[joy_parameters_file],
        remappings=[("/joy", "/drive/joy")],
        condition=IfCondition(use_drive),
    )

    delayed_arm_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                arm_launch,
            ],
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_arm_cmd)
    ld.add_action(declare_use_drive_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_spawners)
    ld.add_action(delayed_arm_launch)
    ld.add_action(drive_control_node)
    return ld
