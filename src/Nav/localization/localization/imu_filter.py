import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class imu_filter(Node):
    def __init__(self):
        super().__init__("imu_filter")
        self.imu_pub = self.create_publisher(Imu, "imu/filtered", 1)
        self.wheel_encoder_sub = self.create_subscription(
            Odometry, "/dev/roboclaws/mid/odom_roboclaw", self.encoder_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/ouster/imu", self.imu_callback, 10
        )
        self.last_encoder_msg = Odometry()

    def encoder_callback(self, msg):
        self.last_encoder_msg = msg

    def imu_callback(self, msg):
        now = self.get_clock().now().nanoseconds()
        message_time_nano = (
            self.last_encoder_msg.header.sec * 10e9
            + self.last_encoder_msg.header.nanosec
        )
        if now - message_time_nano > 1 * 10e9:
            return
        if (
            self.last_encoder_msg.twist.twist.linear.x == 0
            and self.last_encoder_msg.twist.twist.angular.z == 0
        ):
            return
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = imu_filter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
