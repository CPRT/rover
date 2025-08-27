import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Int8

import board
import neopixel_spi as neopixel


class gpioManager(Node):
    def __init__(self):
        super().__init__("gpioNode")
        self.lastTimestamp = 0
        self.gpiooutput = False
        self.lightColor = 0x000000
        self.load_parameters()
        self.get_logger().info(
            "GPIO Manager started with " + str(self.NUM_PIXELS) + " pixels"
        )

        PIXEL_ORDER = neopixel.GRB

        spi = board.SPI()

        self.pixels = neopixel.NeoPixel_SPI(
            spi, self.NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False
        )
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.light_subscriber = self.create_subscription(
            Int8, "/light", self.neoCallback, qos_profile
        )
        self.timer = None
        self.flash = False
        self.flash_state_on = False

    def load_parameters(self):
        self.declare_parameter("num_pixels", 20)
        self.NUM_PIXELS = self.get_parameter("num_pixels").value
        self.declare_parameter("frequency", 1)
        self.period = 1 / self.get_parameter("frequency").value

    def flashing_handler(self):
        if not self.flash:
            self.timer.cancel()
            self.timer = None
            self.pixels.fill(self.lightColor)
            self.flash_state_on = False
        if not self.flash_state_on:
            self.pixels.fill(self.lightColor)
        else:
            self.pixels.fill(0)
        self.flash_state_on = not self.flash_state_on
        self.pixels.show()

    def neoCallback(self, msg: Int8):
        self.get_logger().info("outputting color " + str(msg.data))

        if msg.data == 1:  # red
            self.lightColor = 0xFF0000
            self.flash = False
        elif msg.data == 2:  # blue
            self.lightColor = 0x0000FF
            self.flash = False
        elif msg.data == 3:  # green flashing
            self.lightColor = 0x00FF00
            self.flash = True
        elif msg.data == 0:  # empty
            self.lightColor = 0x000000
            self.flash = False
        else:
            self.get_logger().info("Invalid color code received: " + str(msg.data))
            return

        self.pixels.fill(self.lightColor)
        self.pixels.show()
        if self.flash and self.timer is None:
            self.timer = self.create_timer(self.period, self.flashing_handler)


def main(args=None):
    rclpy.init(args=args)
    node = gpioManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
