import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
import time


class OdometryRepublisher(Node):
    def __init__(self, frequency: float, timeout_duration: float = 2.0):
        super().__init__("odometry_republisher")

        # Set the frequency for republishing (Hz)
        self.frequency = frequency
        self.timeout_duration = Duration(
            seconds=timeout_duration, nanoseconds=0
        )  # Timeout duration in seconds
        self.last_odometry = None
        self.last_received_time = None

        # Flag to track if we've already warned about missing odometry
        self.warned_once = False

        # Subscriber to the original odometry topic
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            "/gps/odom",  # Original Odometry topic /gps/odom
            self.odometry_callback,
            10,  # QoS profile
        )

        # Publisher to the new republished odometry topic
        self.odometry_publisher = self.create_publisher(
            Odometry, "/repub_gps_odom", 10  # New Odometry topic  # QoS profile
        )

        # Timer to republish the last known odometry message at the specified frequency
        self.timer = self.create_timer(1.0 / self.frequency, self.republish_odometry)

    def odometry_callback(self, msg: Odometry):
        # Store the most recent odometry message and update the timestamp
        # self.get_logger().info(f"Received new Odometry message: {msg.header.stamp}")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.last_odometry = msg

        # Update the time of last received message
        self.last_received_time = self.get_clock().now().to_msg()

        # Reset warning flag when a new message is received
        self.warned_once = False

    def republish_odometry(self):
        if self.last_odometry is not None:
            # Check if the last received odometry message was within the timeout duration
            time_diff = self.get_clock().now() - rclpy.time.Time.from_msg(
                self.last_received_time
            )
            if time_diff > self.timeout_duration:
                self.get_logger().info(
                    "No new Odometry message received for 2 seconds. Stopping republishing."
                )
                self.last_odometry = None
            else:
                # Republish the last known odometry message
                self.odometry_publisher.publish(self.last_odometry)
        else:
            # Publish the warning message only once
            if not self.warned_once:
                self.get_logger().warn(
                    "No new Odometry message received yet. Republishing the last one."
                )

                # Set the flag to ensure the warning is only published once
                self.warned_once = True


def main(args=None):
    rclpy.init(args=args)

    # Set the frequency (in Hz) at which to republish the odometry
    frequency = 10  # Change to your desired frequency (Hz)

    # Create the node and start spinning
    node = OdometryRepublisher(frequency)
    rclpy.spin(node)

    # Shutdown ROS 2 when exiting
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
