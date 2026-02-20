#!/usr/bin/env python3
import rclpy
from .roboclaw_library import Roboclaw
from rclpy.node import Node
from std_msgs.msg import Float32


class RoboClaw(Node):
    def __init__(self):
        super().__init__("elevator_roboclaw_node")
        self.get_logger().info("Elevator RoboClaw node has been started.")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        port = self.get_parameter("port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.controller = Roboclaw(port, baud_rate)
        self.address = 0x80  # Default address for the RoboClaw
        if not self.controller.Open():
            self.get_logger().error(f"Failed to connect to RoboClaw on {port}.")
            raise RuntimeError("Could not open serial connection to RoboClaw.")
        self.get_logger().info(f"Connected to RoboClaw on {port} at {baud_rate} baud.")
        self.cmd_sub = self.create_subscription(
            Float32, "elevator/set", self.cmd_callback, 10
        )
        (rc, version) = self.controller.ReadVersion(self.address)
        if rc:
            self.get_logger().info(f"Version: {version}")
        else:
            self.get_logger().error("Failed to read version from RoboClaw.")
        self.telemetry_timer = self.create_timer(10.0, self.telemetry_callback)
        self.watchdog_flag = False
        self.log_flag = True
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

    def telemetry_callback(self):
        (okE, err) = self.controller.ReadError(self.address)
        if okE:
            if err:
                self.get_logger().error(f"Error code: {err}")
        else:
            self.get_logger().error("Failed to read error code from RoboClaw.")

    def watchdog_callback(self):
        if self.watchdog_flag:
            self.watchdog_flag = False
            self.log_flag = True
            return
        if self.log_flag:
            self.get_logger().warn(
                "Watchdog timeout: no command received in the last 0.5 seconds. Stopping motor."
            )
        self.log_flag = False
        if not self.controller.DutyM1(self.address, 0):
            self.get_logger().error("Failed to send watchdog command to RoboClaw.")

    def cmd_callback(self, msg):
        duty_cycle = msg.data
        if duty_cycle < -1.0 or duty_cycle > 1.0:
            self.get_logger().error(
                f"Received invalid duty cycle {duty_cycle}. Must be between -1.0 and 1.0."
            )
            return
        duty_cycle_int16 = int(
            duty_cycle * 32767
        )  # Scale from -1.0 to 1.0 range to -32767 to 32767
        if not self.controller.DutyM1(self.address, duty_cycle_int16):
            self.get_logger().error(
                f"Failed to set duty cycle {duty_cycle_int16} on RoboClaw."
            )
        self.watchdog_flag = True


def main(args=None):
    """
    Initializes the ROS 2 python library, starts this node, and enters the
    ROS 2 spin loop to process incoming messages and trigger callbacks.
    """
    rclpy.init(args=args)
    node = RoboClaw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
