import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo

import random


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")
        self.declare_parameter("port", 0)
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.get_logger().info(f"{self.port}")

        self.cli = self.create_client(MoveServo, "servo_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.timer = self.create_timer(2, self.servo_tester)

    def send_request(self, port: int, pos: int) -> MoveServo:
        req = MoveServo.Request()
        req.port = port
        req.pos = pos
        future = self.cli.call_async(req)

    def servo_request(self, req_port, req_pos) -> None:
        Servo_Client.get_logger(self).info("Sending Request for: %s" % (req_pos))
        self.send_request(port=req_port, pos=req_pos)

    def servo_tester(self) -> None:
        random_pos = random.randint(0, 180)

        self.servo_request(self.port, random_pos)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
