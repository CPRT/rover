#!/usr/bin/env python3
"""Run after sourcing the workspace. Exercises the installed launch and both plugins.

python3 test_e2e.py [--serial-emulator]
The emulator verifies actual serial packets through a PTY, not physical motion.
"""
import argparse
import math
import os
import pty
import select
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-emulator", action="store_true")
    parser.add_argument(
        "--drive-executable", help="Also verify joystick-to-joint integration"
    )
    args = parser.parse_args()
    packets = []
    stop = threading.Event()
    master = slave = None
    launch_args = []
    if args.serial_emulator:
        master, slave = pty.openpty()
        launch_args = [
            "gimbal_mock_hardware:=false",
            "gimbal_serial_port:=" + os.ttyname(slave),
        ]

        def receive():
            pending = b""
            while not stop.is_set():
                if select.select([master], [], [], 0.1)[0]:
                    pending += os.read(master, 4096)
                    while len(pending) >= 6:
                        packet, pending = pending[:6], pending[6:]
                        assert packet[:3] == bytes([0xAA, 12, 4]), packet
                        packets.append((packet[3], packet[4] + (packet[5] << 7)))

        receiver = threading.Thread(target=receive, daemon=True)
        receiver.start()
    rclpy.init()
    node = rclpy.create_node("gimbal_e2e")
    state = {}
    node.create_subscription(
        JointState,
        "/joint_states",
        lambda msg: state.update(zip(msg.name, msg.position)),
        10,
    )
    velocity = node.create_publisher(Twist, "/gimbal_controller/cmd_vel", 1)
    reset = node.create_publisher(Bool, "/gimbal_controller/reset", 1)
    names = ["gimbal_pan_joint", "gimbal_tilt_joint", "mast_camera_joint"]

    def spin(seconds, command=None, home=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if command is not None:
                velocity.publish(command)
            if home is not None:
                reset.publish(Bool(data=home))
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.01)

    def positions():
        return [state[n] for n in names]

    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "rover_urdf", "gimbal.launch.py", *launch_args],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        drive_process = None
        try:
            deadline = time.monotonic() + 40
            while (
                not all(n in state for n in names)
                or velocity.get_subscription_count() == 0
            ) and time.monotonic() < deadline:
                assert process.poll() is None, "launch exited"
                spin(0.1)
            assert all(n in state for n in names), state
            spin(0.5)
            assert max(abs(v) for v in positions()) < 1e-6, positions()
            if args.drive_executable:
                drive_process = subprocess.Popen(
                    [
                        args.drive_executable,
                        "--ros-args",
                        "-r",
                        "__node:=drive_teleop_node",
                        "--params-file",
                        os.path.join(
                            os.path.dirname(__file__),
                            "../../joystick_control/config/8bitdo.yaml",
                        ),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                joy_pub = node.create_publisher(Joy, "/joy", 1)
                joy = Joy(axes=[0.0] * 8, buttons=[0] * 11)

                def joystick(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        joy_pub.publish(joy)
                        spin(0.04)

                joystick(1.0)  # Discovery and neutral-stick initialization.
                joy.axes[6], joy.axes[7], joy.buttons[3] = 0.5, 1.0, 1
                joystick(0.8)
                pan, tilt, mast = positions()
                assert pan > 0.10 and tilt < -0.20 and mast < -0.20, positions()
                joy.buttons[0] = 1
                joystick(1.0)
                assert max(abs(v) for v in positions()) < 0.006, positions()
                joy = Joy(axes=[0.0] * 8, buttons=[0] * 11)
                joystick(0.2)
                joy = Joy()  # Malformed input must not crash teleop.
                joystick(0.2)
                assert (
                    drive_process.poll() is None
                ), "drive crashed on short Joy message"
                drive_process.send_signal(signal.SIGINT)
                drive_process.wait(timeout=5)
                spin(0.6)
            cmd = Twist()
            cmd.angular.z, cmd.angular.y, cmd.angular.x = 0.3, -0.2, 0.1
            spin(1.0, cmd, False)
            pan, tilt, mast = positions()
            assert (
                0.20 < pan < 0.40 and -0.30 < tilt < -0.13 and 0.06 < mast < 0.15
            ), positions()
            # Lost publisher stops motion after timeout and holds the resulting position.
            spin(0.8)
            held = positions()
            spin(0.4)
            assert max(abs(a - b) for a, b in zip(held, positions())) < 0.006
            # Reset takes priority over continuous motion; false cannot cancel a queued home.
            spin(1.5, cmd, True)
            assert max(abs(v) for v in positions()) < 0.006, positions()
            spin(0.2, Twist(), False)
            # Nonfinite angular input stops all axes.
            cmd.angular.z = math.nan
            spin(0.3, cmd)
            assert max(abs(v) for v in positions()) < 0.006, positions()
            # Huge velocities are rate limited and saturate at URDF bounds.
            cmd.angular.z = cmd.angular.y = cmd.angular.x = 100.0
            spin(0.4, cmd)
            assert all(0.13 < v < 0.27 for v in positions()), positions()
            spin(6.3, cmd)
            for actual, limit in zip(positions(), [3.18, 2.53, 2.08]):
                assert abs(actual - limit) < 0.015, positions()
            # A short true/false pulse still completes homing.
            reset.publish(Bool(data=True))
            spin(0.1)
            reset.publish(Bool(data=False))
            spin(6.7)
            assert max(abs(v) for v in positions()) < 0.006, positions()
            # Deactivation/reactivation must discard preactivation commands.
            cmd.angular.z, cmd.angular.y, cmd.angular.x = 0.2, 0.0, 0.0
            spin(0.3, cmd)
            for action in ["--deactivate", "--activate"]:
                result = subprocess.run(
                    [
                        "ros2",
                        "control",
                        "switch_controllers",
                        action,
                        "gimbal_controller",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                assert result.returncode == 0, result.stdout + result.stderr
            spin(0.2)
            held = positions()
            spin(0.7)
            assert max(abs(a - b) for a, b in zip(held, positions())) < 0.006
            if args.serial_emulator:
                assert len(packets) > 100, len(packets)
                assert {channel for channel, _ in packets} == {0, 1, 3}
                assert all(1400 <= target <= 10000 for _, target in packets)
                assert receiver.is_alive(), "serial parser failed"
            print(
                "PASS: launch, plugin loading, three-axis motion, timeout, reset priority/pulse, NaN, velocity/position limits, lifecycle"
                + (
                    f", {len(packets)} serial packets"
                    if args.serial_emulator
                    else ", mock hardware"
                )
            )
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if drive_process is not None and drive_process.poll() is None:
                drive_process.send_signal(signal.SIGINT)
                drive_process.wait(timeout=5)
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
            stop.set()
            if master is not None:
                receiver.join(timeout=1)
                os.close(master)
                os.close(slave)


if __name__ == "__main__":
    main()
