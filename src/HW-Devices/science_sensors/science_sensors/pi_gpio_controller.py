import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class GPIO_Controller(Node):
    def __init__(self):
        super().__init__("gpio_service_node")
        GPIO.setmode(GPIO.BCM)
        self.load_parameters()
        for pin in self.gpio_pins:
            GPIO.setup(pin, GPIO.OUT)
        self.srv = self.create_service(
            SetBool, self.service_name, self.set_gpio_callback
        )

    def load_parameters(self):
        self.declare_parameter(
            "gpio_pins",
            [0],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY),
        )
        self.gpio_pins = (
            self.get_parameter("gpio_pins").get_parameter_value().integer_array_value
        )
        if not self.gpio_pins:
            self.get_logger().error("No GPIO pins provided")
            raise ValueError("No GPIO pins provided")
        self.declare_parameter("service_name", "set_gpio")
        self.service_name = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )

    def set_gpio_callback(self, request, response):
        value = GPIO.HIGH if request.data else GPIO.LOW
        for pin in self.gpio_pins:
            GPIO.output(pin, value)
        response.success = True
        response.message = (
            f"GPIO pin(s) ({self.gpio_pins}) {'ON' if request.data else 'OFF'}"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GPIO_Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
