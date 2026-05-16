"""Science stack: Phoenix drill/elevator, Thrustmaster joy, drill teleop."""

from __future__ import annotations

import glob
import os
import re
import stat
from typing import Iterable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Ordered: first match wins. Thrustmaster sticks often expose *-event-joystick in by-id.
DEFAULT_JOY_GLOBS: Tuple[str, ...] = (
    "/dev/input/by-id/usb-Thrustmaster*event-joystick",
    "/dev/input/by-id/usb-Thrustmaster*joystick",
)


def _event_to_js_device(event_basename: str) -> Optional[str]:
    """Map eventN to /dev/input/jsM using /proc/bus/input/devices."""
    try:
        with open("/proc/bus/input/devices", "r", encoding="utf-8") as f:
            content = f.read()
    except OSError:
        return None
    for block in content.strip().split("\n\n"):
        for line in block.split("\n"):
            if not line.startswith("H: Handlers="):
                continue
            rest = line.split("Handlers=", 1)[1].strip()
            parts = rest.split()
            if event_basename not in parts:
                continue
            for part in parts:
                if re.fullmatch(r"js\d+", part):
                    return f"/dev/input/{part}"
    return None


def _resolve_joy_js_path(patterns: Iterable[str]) -> Optional[str]:
    """
    Find a usable /dev/input/js* path.

    ``joy_node``'s ``device_id`` breaks when by-id symlinks point at event* nodes.
    ``joy_linux_node`` wants a concrete path; we normalize to js* when possible.
    """
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            if not os.path.exists(path):
                continue
            real = os.path.realpath(path)
            base = os.path.basename(real)
            candidate: Optional[str]
            if base.startswith("js"):
                candidate = real
            elif base.startswith("event"):
                candidate = _event_to_js_device(base)
            else:
                continue
            if not candidate or not os.path.exists(candidate):
                continue
            try:
                mode = os.stat(candidate).st_mode
            except OSError:
                continue
            if stat.S_ISCHR(mode):
                return candidate
    return None


def _validate_joy_dev(path: str) -> None:
    if not os.path.exists(path):
        raise RuntimeError(
            f"Joystick device path does not exist: {path!r}. "
            "Set launch arg joy_dev:=/dev/input/jsN, env SCIENCE_JOY_DEV, or plug in the stick."
        )
    try:
        mode = os.stat(path).st_mode
    except OSError as exc:
        raise RuntimeError(f"Cannot stat joystick device {path!r}: {exc}") from exc
    if not stat.S_ISCHR(mode):
        raise RuntimeError(
            f"{path!r} is not a character device (mode {mode:#o}). "
            "In dev containers, mount the host's full /dev/input (not only by-id) "
            "so ../eventN / jsN nodes resolve, or pass joy_dev:= to a valid path."
        )


def _science_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("joystick_control")
    parameters_file = os.path.join(pkg, "pxn.yaml")

    can_iface = LaunchConfiguration("phoenix_can_interface").perform(context)
    drill_id = int(LaunchConfiguration("drill_motor_id").perform(context))
    elev_id = int(LaunchConfiguration("elevator_motor_id").perform(context))

    joy_arg = LaunchConfiguration("joy_dev").perform(context).strip()
    env_dev = os.environ.get("SCIENCE_JOY_DEV", "").strip()

    if joy_arg:
        joy_dev = os.path.realpath(joy_arg)
    elif env_dev:
        joy_dev = os.path.realpath(env_dev)
    else:
        patterns = (
            tuple(
                p.strip()
                for p in os.environ.get("SCIENCE_JOY_GLOB", "").split(":")
                if p.strip()
            )
            or DEFAULT_JOY_GLOBS
        )
        joy_dev = _resolve_joy_js_path(patterns)

    if not joy_dev:
        raise RuntimeError(
            "No joystick found for science teleop. Tried by-id patterns "
            f"{DEFAULT_JOY_GLOBS}. Fix: (1) plug in the Thrustmaster, "
            "(2) launch with joy_dev:=/dev/input/jsN, "
            "(3) set SCIENCE_JOY_DEV or colon-separated SCIENCE_JOY_GLOB, "
            "(4) in devcontainer mount host /dev/input (see .devcontainer)."
        )

    _validate_joy_dev(joy_dev)
    print(f"[rover_science] Using joystick device: {joy_dev}", flush=True)

    return [
        ComposableNodeContainer(
            name="PhoenixContainerScienceTeleop",
            namespace="",
            package="ros_phoenix",
            executable="phoenix_container",
            parameters=[{"interface": can_iface}],
            composable_node_descriptions=[
                ComposableNode(
                    package="ros_phoenix",
                    plugin="ros_phoenix::TalonSRX",
                    name="elevator",
                    parameters=[
                        {"id": elev_id},
                        {"max_voltage": 24.0},
                        {"brake_mode": True},
                    ],
                ),
                ComposableNode(
                    package="ros_phoenix",
                    plugin="ros_phoenix::TalonSRX",
                    name="drill",
                    parameters=[
                        {"id": drill_id},
                        {"max_voltage": 24.0},
                        {"brake_mode": True},
                    ],
                ),
            ],
            output="screen",
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node_b",
            parameters=[
                {
                    "dev": joy_dev,
                    "deadzone": 0.0,
                }
            ],
            remappings=[("/joy", "/drill/joy")],
        ),
        Node(
            package="joystick_control",
            executable="drill",
            name="drill_teleop_node",
            parameters=[parameters_file],
            remappings=[("/joy", "/drill/joy")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_dev",
                default_value="",
                description="Absolute path to joystick (e.g. /dev/input/js0). "
                "If empty, auto-detect Thrustmaster via /dev/input/by-id.",
            ),
            DeclareLaunchArgument(
                "phoenix_can_interface",
                default_value="can0",
            ),
            DeclareLaunchArgument("drill_motor_id", default_value="20"),
            DeclareLaunchArgument("elevator_motor_id", default_value="21"),
            OpaqueFunction(function=_science_setup),
        ]
    )
