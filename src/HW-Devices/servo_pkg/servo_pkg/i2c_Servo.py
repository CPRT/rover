import rclpy
from std_msgs.msg import Float32
import math
from std_msgs.msg import Float32
import math
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from servo_pkg.parent_config import Parent_Config
from servo_pkg.parent_config import Parent_Config


class i2c_Servo(Parent_Config):
class i2c_Servo(Parent_Config):
    def __init__(self):
        super().__init__("i2c_servo")

        self.sub = self.create_subscription(
            Float32,
            f"/{self.servo_info[self.servo_num].motor_name}",
            self.set_position,
            3,
        )

        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        self.maxrom = math.pi  # max range of motion of the servo, default pi
        self.maxrom = math.pi  # max range of motion of the servo, default pi

    def set_position(self, msg):
        if self.servo_list[self.servo_num - 1] == None:
            self.servo_list[self.servo_num] = servo.Servo(
                self.pca.channels[self.servo_num], actuation_range=self.maxrom
    def set_position(self, msg):
        if self.servo_list[self.servo_num - 1] == None:
            self.servo_list[self.servo_num] = servo.Servo(
                self.pca.channels[self.servo_num], actuation_range=self.maxrom
            )
        cur_servo = self.servo_list[self.servo_num]
        cur_servo.angle = msg.data
        self.get_logger().info(
            f"Servo {self.servo_num} moving to {cur_servo.angle} degrees"
        )
        cur_servo = self.servo_list[self.servo_num]
        cur_servo.angle = msg.data
        self.get_logger().info(
            f"Servo {self.servo_num} moving to {cur_servo.angle} degrees"
        )


def main(args=None):
    rclpy.init(args=args)
    node = i2c_Servo()
    rclpy.spin(node)
    node.pca.deinit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
