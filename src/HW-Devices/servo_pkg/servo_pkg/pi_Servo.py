import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo

from rpi_hardware_pwm import HardwarePWM


def to_channel(pin: int) -> int:
    if pin == 18:
        return 0
    elif pin == 19:
        return 1
    raise ValueError(f"Entered non PWM pin: {pin}")


class Servo:
    def __init__(
        self, channel: int, min_pos: float, max_pos: float, frequency: int, rom: int
    ):
        self.channel = channel
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.frequency = frequency
        self.rom = rom

        self.pwm_pin = HardwarePWM(pwm_channel=self.channel, hz=self.frequency, chip=0)
        self.pwm_pin.start(0)

    def set_position(self, degree: int):
        if degree < 0 or degree > self.rom:
            raise ValueError(f"Degree out of range: {degree}")

        duty_cycle = self.convert_to_pwm(degree)
        self.pwm_pin.change_duty_cycle(duty_cycle)

    def convert_to_pwm(self, degree: int) -> float:
        return float(degree / (self.rom / (self.max_pos - self.min_pos)) + self.min_pos)

    def stop(self):
        self.pwm_pin.stop()


class pi_Servo(Node):
    def __init__(self):
        super().__init__("pi_servo")
        self.srv = self.create_service(MoveServo, "servo_service", self.set_position)
        self.servos = {}
        self.load_params()

    def load_params(self):
        self.declare_parameter("servos_used", 0)
        num_servos = (
            self.get_parameter("servos_used").get_parameter_value().integer_value
        )
        if num_servos <= 0:
            self.get_logger().error("Invalid number of ports")
            raise ValueError("Invalid number of ports")

        for i in range(num_servos):
            self.declare_parameter(f"servo{i}.frequency", 50)
            self.declare_parameter(f"servo{i}.min", 0.0)
            self.declare_parameter(f"servo{i}.max", 100.0)
            self.declare_parameter(f"servo{i}.rom", 180)
            self.declare_parameter(f"servo{i}.out_pin", 0)
            frequency = (
                self.get_parameter(f"servo{i}.frequency")
                .get_parameter_value()
                .integer_value
            )
            min_pos = (
                self.get_parameter(f"servo{i}.min").get_parameter_value().double_value
            )
            max_pos = (
                self.get_parameter(f"servo{i}.max").get_parameter_value().double_value
            )
            rom = (
                self.get_parameter(f"servo{i}.rom").get_parameter_value().integer_value
            )
            outpin = (
                self.get_parameter(f"servo{i}.out_pin")
                .get_parameter_value()
                .integer_value
            )
            if outpin < 0:
                self.get_logger().error(f"Invalid pin number for port {i}")
                raise ValueError(f"Invalid pin number for port {i}")

            self.servos[i] = Servo(
                channel=to_channel(outpin),
                min_pos=min_pos,
                max_pos=max_pos,
                frequency=frequency,
                rom=rom,
            )

    def set_position(self, request, response) -> MoveServo:
        port = request.port
        if port not in self.servos:
            response.status = False
            response.status_msg = f"Invalid port: {port}"
            self.get_logger().error(response.status_msg)
            return response

        servo = self.servos[port]
        degree = request.pos
        try:
            servo.set_position(degree)
        except ValueError as e:
            response.status = False
            response.status_msg = str(e)
            self.get_logger().error(f"Error setting position {str(e)}")
            return response
        response.status = True
        response.status_msg = f"Moved to {degree} degrees"
        self.get_logger().info(response.status_msg)
        return response

    def destroy_node(self):
        for servo in self.servos.values():
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
