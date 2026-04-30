#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from .roboclaw_library import Roboclaw


class RoboClawAntennaNode(Node):
    def __init__(self):
        super().__init__("roboclaw_antenna_node")
        self.get_logger().info("RoboClaw Antenna node started.")

        # Parameters 
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("max_speed", 700)
        self.declare_parameter("accel", 350)
        self.declare_parameter("enc_read_freq", 10.0)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud_rate").value
        self.address = 0x80 # default address for Roboclaw
        self.counts_per_rev = 4096 * 2 # gear ratio of 2:1 and 4096 count for full encoder rev

        self.max_speed = self.get_parameter("max_speed").value
        self.accel = self.get_parameter("accel").value
        self.enc_read_freq = self.get_parameter("enc_read_freq").value

        #  RoboClaw Setup 
        self.controller = Roboclaw(port, baud)

        if not self.controller.Open():
            raise RuntimeError(f"Failed to open RoboClaw on {port}")

        self.get_logger().info(f"Connected to RoboClaw on {port}")

        rc, version = self.controller.ReadVersion(self.address)
        if rc:
            self.get_logger().info(f"Firmware: {version}")
        else:
            self.get_logger().warn("Could not read firmware version")

        # Reset encoder (likely when facing north until imu setup)
        self.controller.SetEncM1(self.address, 0)
        # Hold state of encoder and values
        self.current_encoder = 0
        self.target_encoder = 0
        self.zero_offset = 0

        # ROS Interfaces 
        self.create_subscription(
            Float32, "/antenna/bearing", self.delta_callback, 10
        )
        self.create_timer(1.0 / self.enc_read_freq, self.encoder_timer)

    # Angle to Encoder Command
    def delta_callback(self, msg: Float32):
        deg = msg.data % 360

        self.target_encoder = self.zero_offset + int((deg / 360) * self.counts_per_rev)

        error = self.wrap_error(self.target_encoder - self.current_encoder)
    
        # ------ try with testing tomorrow ------
        # deadband
        # if abs(error) < 20:
            # return

        # smoothing
        # step = int(error * 0.3)

        # max_step = 2000
        # step = max(-max_step, min(max_step, step))

        # command = self.current_encoder + step

        self.get_logger().info(
            f"target= {deg:.2f}°, {self.target_encoder}"
        )

        self.drive_to_position(self.current_encoder + error)

    def wrap_error(self, error):
        half = self.counts_per_rev // 2
        if error > half:
            error -= self.counts_per_rev
        elif error < -half:
            error += self.counts_per_rev
        return error
    
    # Encoder Reading - If we want to check for encoder delta
    def encoder_timer(self):
        rc, enc, _ = self.controller.ReadEncM1(self.address)

        if not rc:
            self.get_logger().warn("Failed to read encoder")
            return

        self.current_encoder = enc
    
    # Position Control
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

    # Shutdown Safety    
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
