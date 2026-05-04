import rclpy
from std_msgs.msg import Float32
from servo_pkg import maestro
from servo_pkg.parent_config import Parent_Config


def convert_from_radians(angle, servo_info):
    total_range = servo_info.max - servo_info.min
    return int(servo_info.min + (total_range * angle / servo_info.rom))


def convert_to_radians(value, servo_info):
    total_range = servo_info.max - servo_info.min
    return servo_info.rom * (value - servo_info.min) / total_range


class USB_Servo(Parent_Config):
    def __init__(self):
        super().__init__("usb_servo", 12)

        # port parameter
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.servo_controller = maestro.Controller(serial_port)
        self.subs = {}
        for port in self.servo_info:
            self.subs[port] = self.create_subscription(
                Float32,
                f"/{self.servo_info[port].motor_name}",
                lambda msg, port=port: self.set_position(msg, port),
                3,
            )
            self.get_logger().info(f"Topic: /{self.servo_info[port].motor_name}")

    def set_position(self, msg, port):
        if not self.check_valid_servo(port):
            return
        servo_info = self.servo_info[port]
        target_value = convert_from_radians(msg.data, servo_info)
        self.get_logger().info(f"Target value: {target_value}")
        current_position = convert_to_radians(
            self.servo_controller.getPosition(port), servo_info
        )

        if not (servo_info.min <= target_value <= servo_info.max):
            self.get_logger().warning(
                f"Servo {port} out of range. {target_value} ({servo_info.min},{servo_info.max})"
            )
        else:
            self.get_logger().debug(
                f"Received request for port {port}: {msg.data} angle -> {target_value}"
            )
            # Maestro servo controller uses quarter micro seconds
            self.servo_controller.setTarget(port, int(4 * target_value))


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
