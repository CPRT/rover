import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO


class GPIOReader(Node):
    def __init__(self):
        super().__init__("gpio_reader_node")
        self.declare_parameter("gpio_pins", [])
        self.declare_parameter("interval", 1.0)

        self.gpio_pins = (
            self.get_parameter("gpio_pins").get_parameter_value().integer_array_value
        )
        self.interval = (
            self.get_parameter("interval").get_parameter_value().double_value
        )

        if not self.gpio_pins:
            self.get_logger().error("No GPIO pins provided")
            raise ValueError("No GPIO pins provided")

        GPIO.setmode(GPIO.BCM)
        for pin in self.gpio_pins:
            GPIO.setup(pin, GPIO.IN)

        self.publishers = {
            pin: self.create_publisher(Bool, f"/gpio/{pin}", 10)
            for pin in self.gpio_pins
        }

        self.timer = self.create_timer(self.interval, self.read_gpio)

        self.get_logger().info(
            f"Reading GPIO pins {self.gpio_pins} every {self.interval} seconds"
        )

    def read_gpio(self):
        for pin in self.gpio_pins:
            value = GPIO.input(pin)
            msg = Bool()
            msg.data = bool(value)
            self.publishers[pin].publish(msg)
            self.get_logger().debug(f"Published GPIO {pin}: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = GPIOReader()
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
