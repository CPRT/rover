#!/usr/bin/env python3
import csv
import os
from datetime import datetime
from threading import Event
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from interfaces.msg import PolarimeterSweep, PwmCommand
from interfaces.srv import RunPolarimeter

TYPE_POLAR = 0x02


class PolarimeterNode(Node):
    def __init__(self):
        super().__init__("polarimeter")
        self.declare_parameter("polar_pin", 27)
        self.declare_parameter("polar_frequency", 50)
        self.declare_parameter("output_dir", "/usr/local/zed/polar/")
        self.declare_parameter("sweep_timeout_s", 90.0)

        self._pin = int(
            self.get_parameter("polar_pin").get_parameter_value().integer_value
        )
        self._frequency = int(
            self.get_parameter("polar_frequency").get_parameter_value().integer_value
        )
        self._output_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self._timeout_s = (
            self.get_parameter("sweep_timeout_s").get_parameter_value().double_value
        )

        self._sweep_event = Event()
        self._latest_sweep: PolarimeterSweep | None = None
        self._service_cb_group = MutuallyExclusiveCallbackGroup()

        self._pwm_pub = self.create_publisher(PwmCommand, "/esp_pwm_command", 10)
        self.create_subscription(
            PolarimeterSweep,
            "/esp_polarimeter_readings",
            self._on_sweep,
            qos_profile=10,
        )
        self.create_service(
            RunPolarimeter,
            "run_polarimeter",
            self._run_polarimeter,
            callback_group=self._service_cb_group,
        )

        os.makedirs(self._output_dir, exist_ok=True)
        self.get_logger().info(
            f"Polarimeter service ready on 'run_polarimeter' "
            f"(pin={self._pin}, freq={self._frequency} Hz, output_dir='{self._output_dir}')"
        )

    def _on_sweep(self, msg: PolarimeterSweep):
        self._latest_sweep = msg
        self._sweep_event.set()

    @staticmethod
    def _write_csv(file_path: str, readings: list[int]) -> None:
        with open(f"{file_path}.csv", "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["angle", "diff"])
            for angle, diff in enumerate(readings):
                writer.writerow([angle, diff])

    @staticmethod
    def _write_graph(title: str, file_path: str, readings: list[int]) -> None:
        x_deg = list(range(len(readings)))
        x_rad = np.deg2rad(x_deg)

        # --- Model: A cos^2(x - phi) + C ---
        def cos2_model(x, A, phi, C):
            return A * np.cos(x - phi) ** 2 + C

        # --- Initial parameter guesses ---
        A_guess = np.max(readings) - np.min(readings)
        phi_guess = np.deg2rad(50)
        C_guess = np.min(readings)

        popt, _ = curve_fit(
            cos2_model, x_rad, readings, p0=[A_guess, phi_guess, C_guess]
        )

        A_fit, phi_fit, C_fit = popt

        # Convert phase to degrees
        phase_deg = np.rad2deg(phi_fit)

        # --- Generate smooth fit curve ---
        x_fit_deg = np.linspace(np.min(x_deg), np.max(x_deg), 1000)
        x_fit_rad = np.deg2rad(x_fit_deg)
        y_fit = cos2_model(x_fit_rad, A_fit, phi_fit, C_fit)

        # --- Plot ---
        plt.figure()
        plt.scatter(x_deg, readings, label="Data")
        plt.plot(x_fit_deg, y_fit, label="Cos² Fit")
        plt.axvline(phase_deg, linestyle="--", label=f"Phase Offset = {phase_deg:.2f}°")

        plt.xlabel("Angle (degrees)")
        plt.ylabel("Signal")
        plt.title(f"{title} - Cosine Squared Fit")
        plt.legend()
        plt.savefig(f"{file_path}.png")

    def _run_polarimeter(self, request, response):
        title = (request.title or "").strip()
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_path = os.path.join(self._output_dir, f"polarimeter_{title}_{timestamp}")

        self._sweep_event.clear()
        self._latest_sweep = None

        cmd = PwmCommand()
        cmd.pin = self._pin & 0xFF
        cmd.type = TYPE_POLAR
        cmd.frequency = self._frequency & 0xFFFF
        self._pwm_pub.publish(cmd)
        self.get_logger().info(
            f"Polarimeter sweep started (pin={cmd.pin}, waiting up to {self._timeout_s}s)"
        )

        if not self._sweep_event.wait(timeout=self._timeout_s):
            response.success = False
            response.message = (
                f"Timed out after {self._timeout_s}s waiting for polarimeter data"
            )
            response.file_path = ""
            return response

        sweep = self._latest_sweep
        if sweep is None or not sweep.readings:
            response.success = False
            response.message = "Received empty polarimeter sweep"
            response.file_path = ""
            return response

        try:
            output_dir = os.path.dirname(os.path.abspath(output_path))
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            self._write_csv(output_path, list(sweep.readings))
            self._write_graph(title, output_path, list(sweep.readings))
        except OSError as exc:
            response.success = False
            response.message = f"Failed to write CSV: {exc}"
            response.file_path = ""
            return response

        response.success = True
        response.message = f"Saved {len(sweep.readings)} samples"
        response.file_path = output_path
        self.get_logger().info(
            f"Polarimeter sweep saved to {output_path} ({len(sweep.readings)} samples)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PolarimeterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
