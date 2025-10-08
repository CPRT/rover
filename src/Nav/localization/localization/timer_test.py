import rclpy
from rclpy.node import Node
import time


class TimerTestNode(Node):
    def __init__(self):
        super().__init__("timer_test_node")
        self.get_logger().info("Timer Test Node Initialized.")
        self.timer1 = self.create_timer(1.0, self.timer_callback_1)
        self.timer2 = self.create_timer(0.5, self.timer_callback_2)

    def timer_callback_1(self):
        self.get_logger().info(
            f"Timer 1 fired at {self.get_clock().now().to_msg().sec}"
        )

    def timer_callback_2(self):
        self.get_logger().info(
            f"Timer 2 fired at {self.get_clock().now().to_msg().sec}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TimerTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
