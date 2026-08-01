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
        self.ser = serial.Serial(
            "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_20:6E:F1:69:EE:E0-if00",
            115200,
        )
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

        self.servo_min = self.get_parameter("min").get_parameter_value().double_value
        self.servo_max = self.get_parameter("max").get_parameter_value().double_value
        self.servo_rom = self.get_parameter("rom").get_parameter_value().double_value

        self.create_timer(0.001, self.loop)
        self.get_logger().info("Mast ESP node started")

    def mast_callback(self, msg: Float32):
        us = convert_from_radians(
            msg.data, self.servo_min, self.servo_max, self.servo_rom
        )
        self.get_logger().debug(f"Received angle: {msg.data:.3f} rad -> PWM: {us} us")
        data = bytes("S" + str(us) + '\n', "utf-8")
        self.ser.write(data)

    def morse_callback(self, msg: String):
        self.get_logger().info(f"Transmitting morse: {msg.data}")
        data = bytes("M" + msg.data + '\n', "utf-8")
        self.ser.write(data)

    def loop(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
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


def main(args=None):
    rclpy.init(args=args)
    node = MastESP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
