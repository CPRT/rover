import rclpy
from rclpy.node import Node
from interfaces.msg import Distance
import serial
from std_msgs.msg import Float32, String


def convert_from_radians(angle, servo_min, servo_max, servo_rom):
    total_range = servo_max - servo_min
    return int(servo_min + (total_range * angle / servo_rom))


class MastESP(Node):
    def __init__(self):
        super().__init__("mast_esp")

        self.pub = self.create_publisher(Distance, "eef_distance", 10)
        self.us_sub = self.create_subscription(
            Float32, "/mast_angle", self.mast_callback, 3
        )
        self.morse_sub = self.create_subscription(
            String, "/morse_transmission", self.morse_callback, 3
        )

        self.declare_parameter("min", 500.0)
        self.declare_parameter("max", 2500.0)
        self.declare_parameter("rom", 6.2832)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter(
            "port",
            "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_20:6E:F1:69:EE:E0-if00",
        )
        self.declare_parameter("baudrate", 115200)

        self._port = self.get_parameter("port").get_parameter_value().string_value
        self._baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        self._reconnect_period = (
            self.get_parameter("reconnect_period_s").get_parameter_value().double_value
        )

        self._serial = None
        self._last_reconnect_attempt_ns = 0

        self.servo_min = self.get_parameter("min").get_parameter_value().double_value
        self.servo_max = self.get_parameter("max").get_parameter_value().double_value
        self.servo_rom = self.get_parameter("rom").get_parameter_value().double_value

        self.create_timer(0.001, self.loop)
        self._ensure_serial_connected(force=True)
        self.get_logger().info(
            f"Mast ESP node started, port={self._port}, baud={self._baud}"
        )

    def _ensure_serial_connected(self, force: bool = False) -> bool:
        if self._serial is not None and self._serial.is_open:
            return True

        now_ns = self.get_clock().now().nanoseconds
        if not force and (now_ns - self._last_connect_attempt_ns) < int(
            self._reconnect_period * 1e9
        ):
            return False
        self._last_connect_attempt_ns = now_ns

        try:
            self._serial = serial.Serial(
                port=self._port, baudrate=self._baudrate, timeout=0.0
            )
            self.get_logger().info(f"Connected to serial port {self._port}")
            return True
        except serial.SerialException as exc:
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

    def mast_callback(self, msg: Float32):
        us = convert_from_radians(
            msg.data, self.servo_min, self.servo_max, self.servo_rom
        )
        self.get_logger().debug(f"Received angle: {msg.data:.3f} rad -> PWM: {us} us")
        data = bytes("S" + str(us) + "\n", "utf-8")
        try:
            self._serial.write(data)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")
            self._close_serial()

    def morse_callback(self, msg: String):
        self.get_logger().info(f"Transmitting morse: {msg.data}")
        data = bytes("M" + msg.data + "\n", "utf-8")
        try:
            self._serial.write(data)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")
            self._close_serial()

    def loop(self):
        if not self._ensure_serial_connected():
            return
        try:
            if self._serial.in_waiting:
                line = self._serial.readline().decode().strip()
                try:
                    reading = int(line)
                    msg = Distance()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    if reading == -2:
                        mm = 0
                        msg.status = Distance.STATUS_ERROR
                    elif reading == -1:
                        mm = 0
                        msg.status = Distance.STATUS_INVALID
                    else:
                        msg.status = Distance.STATUS_OK
                        mm = reading
                        msg.distance = mm / 1000.0
                        self.get_logger().debug(f"Distance: {msg.distance:.3f} m")
                    self.pub.publish(msg)
                except ValueError:
                    pass
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f"Serial read failed: {exc}")
            self._close_serial()
            return

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MastESP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
