import math
import rclpy
from std_msgs.msg import Float32
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from servo_pkg.parent_config import Parent_Config


class i2c_Servo(Parent_Config):
    def __init__(self):
        super().__init__("i2c_servo")

        self.subs = {}
        for servo in self.servo_info:
            self.subs[servo] = self.create_subscription(
                Float32,
                f"/{self.servo_info[servo].motor_name}",
                lambda msg, servo=servo: self.set_position(msg, servo),
                3,
            )

        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        self.maxrom = math.pi  # max range of motion of the servo, default pi

    def set_position(self, msg, port):
        if self.servo_list[port - 1] == None:
            self.servo_list[port] = servo.Servo(
                self.pca.channels[port], actuation_range=self.maxrom
            )
        cur_servo = self.servo_list[port]
        cur_servo.angle = msg.data
        self.get_logger().info(f"Servo {port} moving to {cur_servo.angle} degrees")


def main(args=None):
    rclpy.init(args=args)
    node = i2c_Servo()
    rclpy.spin(node)
    node.pca.deinit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
