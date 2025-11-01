from rclpy.node import Node

DEFAULT_MIN = 512.0
DEFAULT_MAX = 2400.0
DEFAULT_MAX_ANGLE = 3.1415


class Servo_Info:
    def __init__(
        self, motor_name: str, min_pwm: float, max_pwm: float, max_angle: float
    ):
        self.motor_name = motor_name
        self.min = min_pwm
        self.max = max_pwm
        self.rom = max_angle


# Parent class for all 3 types of servos
class Parent_Config(Node):
    def __init__(self, name):
        super().__init__(name)
        self.servo_info = {}
        self.load_config()

    # Set class attributes based on yaml config
    def load_config(self):
        self.declare_parameter("servo_num", 0)
        self.servo_num = (
            self.get_parameter("servo_num").get_parameter_value().integer_value
        )
        # This should be the highest number servo
        self.declare_parameter("max_num_servo", 0)
        self.max_num_servo = (
            self.get_parameter("max_num_servo").get_parameter_value().integer_value
        )
        for servo in range(self.max_num_servo + 1):
            self.declare_parameter(f"servo{servo}.name", f"{servo}")
            motor_name = (
                self.get_parameter(f"servo{servo}.name")
                .get_parameter_value()
                .string_value
            )
            self.declare_parameter(f"servo{servo}.min", DEFAULT_MIN)
            min_pwm = (
                self.get_parameter(f"servo{servo}.min")
                .get_parameter_value()
                .double_value
            )
            self.declare_parameter(f"servo{servo}.max", DEFAULT_MAX)
            max_pwm = (
                self.get_parameter(f"servo{servo}.max")
                .get_parameter_value()
                .double_value
            )
            self.declare_parameter(f"servo{servo}.rom", DEFAULT_MAX_ANGLE)
            rom = (
                self.get_parameter(f"servo{servo}.rom")
                .get_parameter_value()
                .double_value
            )
            self.servo_info[servo] = Servo_Info(motor_name, min_pwm, max_pwm, rom)

    def check_valid_servo(self, channel):
        if self.max_num_servo < 0:
            raise ValueError("Invalid max servo number")
        if channel not in self.servo_info:
            self.get_logger().error("Invalid servo")
            return False
        return True
