import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo
from std_msgs.msg import Float32
from servo_pkg import maestro

NUM_PORTS = 12
DEFAULT_MIN = 512
DEFAULT_MAX = 2400
DEFAULT_MAX_DEGREES = 180


class Servo_Info:
    def __init__(self, port: int, min_us: int, max_us: int, max_deg: int):
        self.port = port
        self.min = min_us
        self.max = max_us
        self.rom = max_deg


def convert_from_degrees(degrees: int, servo_info: Servo_Info) -> int:
    total_range = servo_info.max - servo_info.min
    return int(servo_info.min + (total_range * degrees / servo_info.rom))


def convert_to_degrees(value: int, servo_info: Servo_Info) -> int:
    total_range = servo_info.max - servo_info.min
    return int(servo_info.rom * (value - servo_info.min) / total_range)


class USB_Servo(Node):
    def __init__(self):
        super().__init__("usb_servo")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.servo = maestro.Controller(serial_port)
        self.srv = self.create_service(MoveServo, "servo_service", self.set_position)

        self.servo_ranges = {}
        self.load_port_config()

        for port, servo in self.servo_ranges.items():
            self.servo.setRange(port, servo.min, servo.max)

    def load_port_config(self):
        for port_number in range(NUM_PORTS):
            self.declare_parameter(f"port{port_number}.min", DEFAULT_MIN)
            min_us = (
                self.get_parameter(f"port{port_number}.min")
                .get_parameter_value()
                .integer_value
            )
            self.declare_parameter(f"port{port_number}.max", DEFAULT_MAX)
            max_us = (
                self.get_parameter(f"port{port_number}.max")
                .get_parameter_value()
                .integer_value
            )
            self.declare_parameter(f"port{port_number}.rom", DEFAULT_MAX_DEGREES)
            rom = (
                self.get_parameter(f"port{port_number}.rom")
                .get_parameter_value()
                .integer_value
            )
            # Convert microseconds to quarter-microseconds
            min_qus = min_us * 4
            max_qus = max_us * 4

            self.servo_ranges[port_number] = Servo_Info(
                port_number, min_qus, max_qus, rom
            )
            self.get_logger().info(
                f"Port {port_number} -> Min: {min_qus}, Max: {max_qus}"
            )
        self.declare_parameter("enable_pwm_pin", False)
        enable_pwm_pin = (
            self.get_parameter("enable_pwm_pin").get_parameter_value().bool_value
        )
        if enable_pwm_pin:
            self.declare_parameter("pwm_frequency", 10000)
            self.pwm_frequency = (
                self.get_parameter("pwm_frequency").get_parameter_value().integer_value
            )
            self.pwm_sub = self.create_subscription(
                Float32, "servo_pwm_control", self.pwm_callback, 10
            )

    def pwm_callback(self, msg):
        self.servo.setPwm(duty=msg.data, freq=self.pwm_frequency)

    def set_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        port = request.port
        if port not in self.servo_ranges:
            response.status = False
            response.status_msg = f"Invalid port: {port}"
            return response

        servo_info = self.servo_ranges[port]

        # Update range if explicitly given
        if (
            request.min is not None
            and request.min >= 0
            and request.min < request.max
            and request.max is not None
            and request.max > 0
        ):
            servo_info.min = request.min
            self.servo.setRange(port, servo_info.min, servo_info.max)
            self.get_logger().info(
                f"Updated range for port {port}: Min: {servo_info.min}, Max: {servo_info.max}"
            )

        target_value = convert_from_degrees(request.pos, servo_info)
        current_position = convert_to_degrees(self.servo.getPosition(port), servo_info)

        if not (servo_info.min <= target_value <= servo_info.max):
            response.status = False
            response.status_msg = f"Servo {port} input out of range.\nCurrent position: {current_position}"
        else:
            self.get_logger().debug(
                f"Received request for port {port}: {request.pos} degrees -> {target_value}"
            )
            self.servo.setTarget(port, target_value)
            response.status = True
            current_position = convert_to_degrees(
                self.servo.getPosition(port), servo_info
            )
            response.status_msg = f"Servo {port} moved to: {current_position} degrees"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
