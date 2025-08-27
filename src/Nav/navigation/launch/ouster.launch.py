# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with os_cloud and os_"""

from pathlib import Path
import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import os


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    ouster_ros_pkg_dir = get_package_share_directory("ouster_ros")
    params_file = os.path.join(
        get_package_share_directory("navigation"),
        "config",
        "ouster",
        "driver_params.yaml",
    )

    ouster_ns = LaunchConfiguration("ouster_ns")
    ouster_ns_arg = DeclareLaunchArgument("ouster_ns", default_value="ouster")

    os_driver_name = LaunchConfiguration("os_driver_name")
    os_driver_name_arg = DeclareLaunchArgument(
        "os_driver_name", default_value="os_driver"
    )

    os_driver = LifecycleNode(
        package="ouster_ros",
        executable="os_driver",
        name=os_driver_name,
        namespace=ouster_ns,
        parameters=[params_file],
        output="screen",
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver,
            goal_state="inactive",
            entities=[
                LogInfo(msg="os_driver activating..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(os_driver),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True,
        )
    )

    sensor_finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver,
            goal_state="finalized",
            entities=[
                LogInfo(
                    msg="Failed to communicate with the sensor in a timely manner."
                ),
                EmitEvent(
                    event=launch.events.Shutdown(
                        reason="Couldn't communicate with sensor"
                    )
                ),
            ],
        )
    )

    return launch.LaunchDescription(
        [
            ouster_ns_arg,
            os_driver_name_arg,
            os_driver,
            sensor_configure_event,
            sensor_activate_event,
            sensor_finalized_event,
        ]
    )
