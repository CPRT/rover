import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from pyubx2.ubxtypes_configdb import SET_LAYER_RAM, SET_LAYER_BBR, SET_LAYER_FLASH
from pyubx2 import (
    UBX_PROTOCOL,
)
from .ubx_io_manager import UbxIoManager
import os


class HeadingNode(Node):
    """
    ROS 2 node for ending the via serial communication.

    Attributes:
        heading_pub (Publisher): Publishes Heading messages to the /heading topic.
        serial_conn (IoManager): Manages serial I/O operations.
        layers (int): Configuration layers for the UBXMessage.
        timer (Timer): Timer for periodically reading and publishing heading data.
    """

    def __init__(self):
        """
        Initializes the Heading node, loads parameters, sets up serial communication,
        and starts a periodic timer callback.
        """
        super().__init__("heading_node")
        self.load_params()
        queue_depth = (
            self.get_parameter("QueueDepth").get_parameter_value().integer_value
        )
        self.heading_pub = self.create_publisher(Imu, "/heading", queue_depth)
        self.serial_conn = UbxIoManager(
            port=self.dev, baud=self.baudrate, msg_filter=UBX_PROTOCOL
        )
        self.layers = SET_LAYER_RAM | SET_LAYER_BBR
        if self.persistent:
            self.layers |= SET_LAYER_FLASH
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)

    def quaternion_from_euler(roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q

    def load_params(self):
        """
        Loads parameters from the ROS 2 parameter server with default values.
        Parameters include timing mode, survey-in settings, persistence, frequency,
        baud rate, and serial device.
        """
        self.declare_parameter("Persistent", False)
        self.persistent = (
            self.get_parameter("Persistent").get_parameter_value().bool_value
        )
        self.declare_parameter("Freq", 2.0)
        self.freq = self.get_parameter("Freq").get_parameter_value().double_value
        if self.freq <= 0:
            self.get_logger().warn("Frequency must be positive. Defaulting to 2.0 Hz.")
            self.freq = 2.0

        self.declare_parameter("Baudrate", 115200)
        self.baudrate = (
            self.get_parameter("Baudrate").get_parameter_value().integer_value
        )
        self.declare_parameter("Device", "/dev/ttyUSB0")
        self.dev = self.get_parameter("Device").get_parameter_value().string_value
        self.declare_parameter("QueueDepth", 1)
        self.declare_parameter("frame_id", "gps")
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

    def timer_callback(self):
        """
        Periodic callback to read UBX data from the receiver and publish it to the /heading topic as an IMU.
        If data is available, it publishes the raw data and logs the parsed message.
        """
        while self.serial_conn.data_available():
            raw, parsed_data = self.serial_conn.read()

            if not raw:
                self.get_logger().warn("No data read from serial port.")
                return
            msg = Imu()
            msg.linear_acceleration_covariance[0] = -1
            msg.angular_velocity_covariance[0] = -1
            heading = math.pi / 2 - (parsed_data.relPosHeading / 180.0 * math.pi)
            orientation = HeadingNode.quaternion_from_euler(0, 0, heading)
            msg.orientation.x = orientation[0]
            msg.orientation.y = orientation[1]
            msg.orientation.z = orientation[2]
            msg.orientation.w = orientation[3]
            msg.orientation_covariance[0] = 1000.0
            msg.orientation_covariance[4] = 1000.0
            msg.orientation_covariance[8] = 1000.0
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            # When heading is reported to be valid, use accuracy reported in radiance units
            if parsed_data.relPosValid == 1:
                msg.orientation_covariance[8] = (
                    parsed_data.accHeading / 180.0 * math.pi
                ) ** 2

            self.heading_pub.publish(msg)


def main(args=None):
    """
    Main entry point for the Heading node.

    Initializes the ROS 2 system, creates the Heading node, and starts spinning the event loop.
    """
    rclpy.init(args=args)
    heading_node = HeadingNode()
    rclpy.spin(heading_node)
    heading_node.destroy_node()
    rclpy.shutdown()
