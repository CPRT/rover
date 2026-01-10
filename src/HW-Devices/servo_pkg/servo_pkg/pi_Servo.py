import rclpy
from rpi_hardware_pwm import HardwarePWM
from servo_pkg.parent_config import Parent_Config
from std_msgs.msg import Float32


class pi_Servo_info:
    def __init__(self, channel, servo_info, frequency):
        self.servo_info = servo_info
        self.channel = channel
        self.frequency = frequency
        self.pwm_pin = HardwarePWM(pwm_channel=self.channel, hz=self.frequency, chip=0)
        self.pwm_pin.start(0)

    def set_position(self, angle: float):
        if angle < 0 or angle > self.servo_info.rom:
            raise ValueError(f"Angle out of range: {angle}")
    def set_position(self, angle: float):
        if angle < 0 or angle > self.servo_info.rom:
            raise ValueError(f"Angle out of range: {angle}")

        duty_cycle = self.convert_to_pwm(angle)
        duty_cycle = self.convert_to_pwm(angle)
        self.pwm_pin.change_duty_cycle(duty_cycle)

    def convert_to_pwm(self, angle):
        return float(
            angle / (self.servo_info.rom / (self.max_pos - self.min_pos)) + self.min_pos
        )
    def convert_to_pwm(self, angle):
        return float(
            angle / (self.servo_info.rom / (self.max_pos - self.min_pos)) + self.min_pos
        )

    def stop(self):
        self.pwm_pin.stop()


class pi_Servo(Parent_Config):
class pi_Servo(Parent_Config):
    def __init__(self):
        super().__init__("pi_servo")
        self.subs = {}
        for servo in self.servo_info:
            self.subs[servo] = self.create_subscription(
                Float32,
                f"/{self.servo_info[servo].motor_name}",
                lambda msg, servo=servo: self.set_position(msg, servo),
                3,
            )
        self.servo_list = {}
        self.load_params()

    def load_params(self):
        for servo in self.servo_info:
            self.declare_parameter(f"servo{servo}.frequency", 50)
        for servo in self.servo_info:
            self.declare_parameter(f"servo{servo}.frequency", 50)
            frequency = (
                self.get_parameter(f"servo{servo}.frequency")
                .get_parameter_value()
                .integer_value
            )

            self.servo_list[servo] = pi_Servo_info(
                channel=servo,
                servo_info=self.servo_info[servo],
                frequency=frequency,
            )

    def set_position(self, msg, port):
        servo = self.servo_info[port]
        angle = msg.data
    def set_position(self, msg):
        port = self.servo
        servo = self.servo_info[port]
        angle = msg.data
        try:
            servo.set_position(angle)
            servo.set_position(angle)
        except ValueError as e:
            self.get_logger().error(f"Error setting position {str(e)}")
        self.get_logger().info(f"Moved to angle: {angle}")
        self.get_logger().info(f"Moved to angle: {angle}")

    def destroy_node(self):
        for servo in self.servo_info.values():
        for servo in self.servo_info.values():
            servo.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = pi_Servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
