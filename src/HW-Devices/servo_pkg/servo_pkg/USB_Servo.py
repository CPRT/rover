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
        super().__init__("usb_servo")

        # port parameter
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.servo_controller = maestro.Controller(serial_port)
        self.get_logger().info(f"{self.servo_info[self.servo_num].motor_name}")
        self.sub = self.create_subscription(
            Float32,
            f"/{self.servo_info[self.servo_num].motor_name}",
            self.set_position,
            3,
        )

        self.set_range()
        for port, servo in self.servo_info.items():
            self.servo_controller.setRange(port, servo.min, servo.max)

    def set_range(self):
        for port in self.servo_info:
            # Convert microseconds to quarter-microseconds
            min_qus = self.servo_info[port].min * 4
            max_qus = self.servo_info[port].max * 4
            self.get_logger().info(f"Port {port} -> Min: {min_qus}, Max: {max_qus}")
            self.servo_controller.setRange(port, min_qus, max_qus)

    def set_position(self, msg):
        port = self.servo_num

        self.check_valid_servo(port)
        servo_info = self.servo_info[port]
        total_range = servo_info.max - servo_info.min
        target_value = convert_from_radians(msg.data, servo_info)
        self.get_logger().info(f"Target value: {target_value}")
        current_position = convert_to_radians(
            self.servo_controller.getPosition(port), servo_info
        )

        if not (servo_info.min <= target_value <= servo_info.max):
            self.get_logger().warning(
                f"Servo {port} input out of range.\nCurrent position: {current_position}"
            )
        else:
            self.get_logger().debug(
                f"Received request for port {port}: {msg.data} angle -> {target_value}"
            )
            self.servo_controller.setTarget(port, target_value)
            current_position = convert_to_radians(
                self.servo_controller.getPosition(port), servo_info
            )
            self.get_logger().info(
                f"Servo {port} moved to angle: {current_position} with PWM {target_value}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
