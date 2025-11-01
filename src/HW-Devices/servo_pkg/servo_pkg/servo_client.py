import rclpy
from rclpy.node import Node
import rclpy.logging
import rclpy.time
from std_msgs.msg import Float32
import random
import math


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")

        self.declare_parameter("servo_num", 0)
        self.servo_num = (
            self.get_parameter("servo_num").get_parameter_value().integer_value
        )
        self.declare_parameter(f"servo{self.servo_num}.name", f"servo{self.servo_num}")
        self.motor_name = (
            self.get_parameter(f"servo{self.servo_num}.name")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f"motor name is {self.motor_name}")

        # publish angle with topic as motor name
        self.pub = self.create_publisher(Float32, f"{self.motor_name}", 3)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.servo_tester)

    def servo_pub(self, req_pos) -> None:
        self.get_logger().info(f"Publishing: {req_pos} to {self.motor_name}")
        self.pub.publish(req_pos)

    def servo_tester(self) -> None:
        msg = Float32()
        # random value
        msg.data = random.uniform(0, math.pi)
        # set value
        # msg.data = math.pi / 4
        self.get_logger().info(
            f"Sending position: {msg.data} to {self.motor_name}"
        )
        self.servo_pub(msg)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
