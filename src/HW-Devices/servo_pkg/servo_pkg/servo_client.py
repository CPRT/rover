import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import math


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")

        self.declare_parameter("servo_num", 1)
        self.servo_num = (
            self.get_parameter("servo_num").get_parameter_value().integer_value
        )
        self.declare_parameter(f"servo{self.servo_num}.name", "collection")
        self.motor_name = (
            self.get_parameter(f"servo{self.servo_num}.name")
            .get_parameter_value()
            .string_value
        )

        # publish angle with topic as motor name
        self.pub = self.create_publisher(Float32, f"/{self.motor_name}", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.servo_tester)

    def servo_pub(self, req_pos) -> None:
        self.pub.publish(req_pos)

    def servo_tester(self) -> None:
        msg = Float32()
        # random value
        msg.data = random.uniform(math.pi / 2, math.pi)
        # set value
        self.get_logger().info(f"Sending position: {msg.data} to {self.motor_name}")
        self.servo_pub(msg)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
