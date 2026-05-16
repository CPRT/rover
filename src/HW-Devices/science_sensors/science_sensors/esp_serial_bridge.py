#!/usr/bin/env python3
import os
import struct

import rclpy
from rclpy.node import Node

from interfaces.msg import EspSensorReadings, PwmCommand

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - runtime dependency check
    serial = None
    SerialException = Exception


class EspSerialBridge(Node):
    _PWM_STRUCT = struct.Struct("<BHHHH")
    _SENSOR_STRUCT = struct.Struct("<HHHff")

    _MAGIC0 = 0xAA
    _MAGIC1 = 0x55

    @staticmethod
    def _normalize_port(port_value: str) -> str:
        port = (port_value or "").strip()
        if not port:
            return "/dev/serial/by-id"
        if port.startswith("/"):
            return port
        if port.startswith("tty"):
            return os.path.join("/dev", port)
        return port

    @staticmethod
    def _resolve_port_path(port_value: str) -> str:
        """
        Resolve a usable serial device path.
        - '/dev/serial/by-id' directory -> first entry inside it
        - explicit device path -> returned unchanged
        """
        if os.path.isdir(port_value):
            try:
                entries = sorted(os.listdir(port_value))
            except OSError:
                return port_value
            for entry in entries:
                candidate = os.path.join(port_value, entry)
                if os.path.exists(candidate):
                    return candidate
        return port_value

    @staticmethod
    def _checksum(data: bytes) -> int:
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum & 0xFF

    def __init__(self):
        super().__init__("esp_serial_bridge")

        self.declare_parameter(
            "esp_port",
            "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
        )
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("sensor_topic", "esp_sensor_readings")
        self.declare_parameter("pwm_command_topic", "esp_pwm_command")
        self.declare_parameter("read_poll_hz", 100.0)
        self.declare_parameter("reconnect_period_s", 2.0)

        configured_port = self.get_parameter("esp_port").get_parameter_value().string_value
        self._port_config = self._normalize_port(configured_port)
        self._port = self._resolve_port_path(self._port_config)
        if configured_port != self._port_config:
            self.get_logger().info(
                f"Normalized serial port '{configured_port}' -> '{self._port_config}'"
            )
        if self._port != self._port_config:
            self.get_logger().info(
                f"Resolved serial device from '{self._port_config}' -> '{self._port}'"
            )
        self._baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        sensor_topic = (
            self.get_parameter("sensor_topic").get_parameter_value().string_value
        )
        pwm_command_topic = (
            self.get_parameter("pwm_command_topic").get_parameter_value().string_value
        )
        read_poll_hz = (
            self.get_parameter("read_poll_hz").get_parameter_value().double_value
        )
        reconnect_period = (
            self.get_parameter("reconnect_period_s").get_parameter_value().double_value
        )

        self._serial = None
        self._rx_buffer = bytearray()
        self._reconnect_period = max(0.1, reconnect_period)
        self._last_connect_attempt_ns = 0

        self._sensor_pub = self.create_publisher(EspSensorReadings, sensor_topic, 10)
        self.create_subscription(
            PwmCommand, pwm_command_topic, self._on_pwm_command, qos_profile=10
        )

        poll_period = 1.0 / max(1.0, read_poll_hz)
        self.create_timer(poll_period, self._poll_serial)

        if serial is None:
            self.get_logger().error(
                "pyserial is not installed. Install python3-serial/pyserial."
            )
            return

        self._ensure_serial_connected(force=True)
        self.get_logger().info(
            f"ESP serial bridge ready. cmd_topic='{pwm_command_topic}' "
            f"sensor_topic='{sensor_topic}' port='{self._port}' baud={self._baudrate}"
        )

    def _ensure_serial_connected(self, force: bool = False) -> bool:
        if serial is None:
            return False
        if self._serial is not None and self._serial.is_open:
            return True

        now_ns = self.get_clock().now().nanoseconds
        if not force and (now_ns - self._last_connect_attempt_ns) < int(
            self._reconnect_period * 1e9
        ):
            return False
        self._last_connect_attempt_ns = now_ns

        try:
            resolved_port = self._resolve_port_path(self._port_config)
            if resolved_port != self._port:
                self._port = resolved_port
                self.get_logger().info(f"Using serial device '{self._port}'")
            self._serial = serial.Serial(
                port=self._port, baudrate=self._baudrate, timeout=0.0
            )
            self.get_logger().info(f"Connected to ESP serial port {self._port}")
            return True
        except SerialException as exc:
            self._serial = None
            self.get_logger().warn(
                f"Failed to open serial port {self._port}: {exc}. Retrying..."
            )
            return False

    def _close_serial(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _on_pwm_command(self, msg: PwmCommand):
        if not self._ensure_serial_connected():
            return

        payload = self._PWM_STRUCT.pack(
            int(msg.pin) & 0xFF,
            int(msg.duty_cycle) & 0xFFFF,
            int(msg.duration) & 0xFFFF,
            int(msg.frequency) & 0xFFFF,
            int(msg.ramp) & 0xFFFF,
        )

        checksum = self._checksum(payload)

        packet = (
            bytes(
                [
                    self._MAGIC0,
                    self._MAGIC1,
                ]
            )
            + payload
            + bytes([checksum])
        )

        try:
            self._serial.write(packet)

        except (SerialException, OSError) as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")
            self._close_serial()

    def _poll_serial(self):
        if not self._ensure_serial_connected():
            return

        try:
            available = self._serial.in_waiting
            if available > 0:
                self._rx_buffer.extend(self._serial.read(available))
        except (SerialException, OSError) as exc:
            self.get_logger().warn(f"Serial read failed: {exc}")
            self._close_serial()
            return
        # header(2) + payload + checksum(1)
        packet_size = 2 + self._SENSOR_STRUCT.size + 1

        while len(self._rx_buffer) >= packet_size:
            if self._rx_buffer[0] != self._MAGIC0 or self._rx_buffer[1] != self._MAGIC1:
                del self._rx_buffer[0]
                continue

            payload_start = 2
            payload_end = payload_start + self._SENSOR_STRUCT.size

            payload = bytes(self._rx_buffer[payload_start:payload_end])

            received_checksum = self._rx_buffer[payload_end]
            expected_checksum = self._checksum(payload)

            if received_checksum != expected_checksum:
                del self._rx_buffer[0]
                continue

            (
                methane,
                co2,
                polarimeter,
                temperature,
                moisture,
            ) = self._SENSOR_STRUCT.unpack(payload)

            del self._rx_buffer[:packet_size]

            msg = EspSensorReadings()
            msg.methane = methane
            msg.co2 = co2
            msg.polarimeter = polarimeter
            msg.temperature = temperature
            msg.moisture = moisture
            self._sensor_pub.publish(msg)

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EspSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
