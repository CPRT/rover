#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from .roboclaw_library import Roboclaw


class RoboClawAntennaNode(Node):
    def __init__(self):
        super().__init__("roboclaw_antenna_node")
        self.get_logger().info("RoboClaw Antenna node started.")

        # ---------------- Parameters ----------------
        self.declare_parameter("port", "/dev/ttyS0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("address", 0x80)
        self.declare_parameter("counts_per_rev", 200_000)
        self.declare_parameter("max_speed", 10_000)
        self.declare_parameter("accel", 5_000)
        self.declare_parameter("enc_read_freq", 10.0)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud_rate").value
        self.address = self.get_parameter("address").value

        self.counts_per_rev = self.get_parameter("counts_per_rev").value
        self.max_speed = self.get_parameter("max_speed").value
        self.accel = self.get_parameter("accel").value
        self.enc_read_freq = self.get_parameter("enc_read_freq").value

        # ---------------- RoboClaw Setup ----------------
        self.controller = Roboclaw(port, baud)

        if not self.controller.Open():
            raise RuntimeError(f"Failed to open RoboClaw on {port}")

        self.get_logger().info(f"Connected to RoboClaw on {port}")

        rc, version = self.controller.ReadVersion(self.address)
        if rc:
            self.get_logger().info(f"Firmware: {version}")
        else:
            self.get_logger().warn("Could not read firmware version")

        # Reset encoder
        self.controller.SetEncM1(self.address, 0)

        # ---------------- State ----------------
        self.current_encoder = 0
        self.target_encoder = 0

        # ---------------- ROS Interfaces ----------------
        self.encoder_pub = self.create_publisher(Int32, "/antenna/encoder_counts", 10)

        self.create_subscription(
            Float32, "/antenna/delta_angle", self.delta_callback, 10
        )

        self.create_timer(1.0 / self.enc_read_freq, self.encoder_timer)

    # --------------------------------------------------
    # Angle → Encoder Command
    # --------------------------------------------------

    def delta_callback(self, msg: Float32):
        delta_deg = msg.data

        delta_counts = int((delta_deg / 360.0) * self.counts_per_rev)

        if abs(delta_counts) < 10:
            return

        self.target_encoder += delta_counts

        self.get_logger().info(
            f"Δ{delta_deg:.2f}°, Δ{delta_counts} counts | target={self.target_encoder}"
        )

        self.drive_to_position(self.target_encoder)

    # --------------------------------------------------
    # Encoder Reading
    # --------------------------------------------------

    def encoder_timer(self):
        rc, enc, status = self.controller.ReadEncM1(self.address)

        if not rc:
            self.get_logger().warn("Failed to read encoder")
            return

        self.current_encoder = enc

        msg = Int32()
        msg.data = enc
        self.encoder_pub.publish(msg)

    # --------------------------------------------------
    # Position Control
    # --------------------------------------------------

    def drive_to_position(self, target):
        """
        Uses RoboClaw's built-in position control:
        SpeedAccelDeccelPositionM1(address, accel, speed, deccel, position, buffer)
        """

        success = self.controller.SpeedAccelDeccelPositionM1(
            self.address,
            self.accel,
            self.max_speed,
            self.accel,
            target,
            0,  # 0 = immediate execution
        )

        if not success:
            self.get_logger().error("Failed to send position command")

    # --------------------------------------------------
    # Shutdown Safety
    # --------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Stopping motor on shutdown")
        self.controller.DutyM1(self.address, 0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboClawAntennaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
