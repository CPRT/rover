#!/usr/bin/env python3
"""Validate installed description variants and agreement with controller limits."""
import itertools
import os
import signal
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
import rclpy
from rcl_interfaces.srv import GetParameters

share = Path(get_package_share_directory("rover_urdf"))
config = yaml.safe_load((share / "config/ros2_controllers.yaml").read_text())[
    "gimbal_controller"
]["ros__parameters"]
for arm, drive, gimbal, mock in itertools.product(["false", "true"], repeat=4):
    result = subprocess.run(
        [
            "xacro",
            str(share / "urdf/rover_urdf.urdf.xacro"),
            f"use_arm:={arm}",
            f"use_drive:={drive}",
            f"use_gimbal:={gimbal}",
            f"gimbal_mock_hardware:={mock}",
            "gimbal_mount_xyz:=0.1 0.2 0.3",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    robot = ET.fromstring(result.stdout)
    links = [link.attrib["name"] for link in robot.findall("link")]
    assert len(links) == len(set(links)), "duplicate links"
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    assert len(joints) == len(robot.findall("joint")), "duplicate joints"
    for joint in joints.values():
        assert joint.find("parent").attrib["link"] in links
        assert joint.find("child").attrib["link"] in links
    system = robot.find("ros2_control[@name='GimbalSystem']")
    if gimbal == "false":
        assert system is None and "gimbal_pan_joint" not in joints
        continue
    plugin = system.findtext("hardware/plugin")
    assert plugin == (
        "mock_components/GenericSystem"
        if mock == "true"
        else "gimbal_control/MaestroSystem"
    )
    assert joints["gimbal_mount_joint"].find("origin").attrib["xyz"] == "0.1 0.2 0.3"
    for i, name in enumerate(config["joints"]):
        joint = joints[name]
        assert joint.attrib["type"] == "revolute"
        limit = joint.find("limit")
        assert float(limit.attrib["lower"]) == config["lower_limits"][i]
        assert float(limit.attrib["upper"]) == config["upper_limits"][i]
        assert float(limit.attrib["velocity"]) == config["max_velocity"][i]
        interface = system.find(f"joint[@name='{name}']/command_interface")
        assert interface.attrib["name"] == "position"
        assert (
            float(interface.findtext("param[@name='min']")) == config["lower_limits"][i]
        )
        assert (
            float(interface.findtext("param[@name='max']")) == config["upper_limits"][i]
        )
print(
    "PASS: 16 URDF variants, unique links/joints, valid parents/children, mounting override, controller/hardware/URDF limit agreement"
)

# Check the installed launch path as well as direct xacro invocation: a launch
# argument must actually reach the description consumed by controller_manager.
rclpy.init()
try:
    for enabled in ["false", "true"]:
        node = rclpy.create_node("description_launch_test")
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "rover_urdf", "launch.py", f"use_gimbal:={enabled}"],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                client = node.create_client(
                    GetParameters, "/robot_state_publisher/get_parameters"
                )
                assert client.wait_for_service(
                    timeout_sec=20
                ), "description publisher unavailable"
                future = client.call_async(
                    GetParameters.Request(names=["robot_description"])
                )
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                assert (
                    future.done() and future.result() is not None
                ), "description query timed out"
                robot = ET.fromstring(future.result().values[0].string_value)
                present = enabled == "true"
                assert (
                    robot.find("ros2_control[@name='GimbalSystem']") is not None
                ) == present
                assert (
                    robot.find("joint[@name='gimbal_pan_joint']") is not None
                ) == present
            except BaseException:
                log.seek(0)
                print(log.read())
                raise
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                node.destroy_node()
finally:
    rclpy.shutdown()
print("PASS: installed description launch includes/omits gimbal hardware and joints")
