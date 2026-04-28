import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message as Rtcm
from .ubx_io_manager import UbxIoManager


class RtcmNode(Node):
    """
    ROS 2 node for managing RTCM data via serial communication.

    Attributes:
        rtcm_pub (Publisher): Publishes RTCM messages to the /rtcm topic.
        serial_conn (IoManager): Manages serial I/O operations.
        layers (int): Configuration layers for the UBXMessage.
        timer (Timer): Timer for periodically reading and publishing RTCM data.
    """

    def __init__(self):
        """
        Initializes the RTCM node, loads parameters, sets up serial communication,
        configures RTCM output, and starts a periodic timer callback.
        """
        super().__init__("rtcm_node")
        self.load_params()
        queue_depth = (
            self.get_parameter("QueueDepth").get_parameter_value().integer_value
        )
        self.rtcm_pub = self.create_publisher(Rtcm, "/rtcm", queue_depth)
        self.serial_conn = UbxIoManager(port=self.dev, baud=self.baudrate)
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)

    def load_params(self):
        """
        Loads parameters from the ROS 2 parameter server with default values.
        Parameters include timing mode, survey-in settings, persistence, frequency,
        baud rate, and serial device.
        """
        self.declare_parameter("Freq", 2.0)
        self.freq = self.get_parameter("Freq").get_parameter_value().double_value
        if self.freq <= 0:
            self.get_logger().warn("Frequency must be positive. Defaulting to 2.0 Hz.")
            self.freq = 2.0

        self.declare_parameter("Baudrate", 38400)
        self.baudrate = (
            self.get_parameter("Baudrate").get_parameter_value().integer_value
        )
        self.declare_parameter("Device", "/dev/ttyACM0")
        self.dev = self.get_parameter("Device").get_parameter_value().string_value
        self.declare_parameter("QueueDepth", 1)

    def timer_callback(self):
        """
        Runs in a separate thread, continuously reading from the serial port.
        This is a blocking read, which is efficient as the thread sleeps
        until data is available.
        """
        if self.serial_conn is None:
            self.get_logger().warn("No serial connection . Exiting read loop.")
            return

        try:
            while rclpy.ok():
                # self.serial_conn.read() is a blocking call
                # the threading will sleep here until data is available (0% CPU usage)
                # or the port is closed by destroy_node

                raw, parsed_data = self.serial_conn.read()

                if not raw:
                    # for read timeouts or empty reads, just continue

                    if rclpy.ok():
                        self.get_logger().info("Serial read returned no data.")

                        # short pause to avoid busy loop if read()
                        # this should rarely happen
                        rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                msg = Rtcm()
                msg.message = list(raw)

                self.rtcm_pub.publish(msg)
                self.get_logger().debug(
                    f"Published RTCM message of length {len(raw)}. Parsed message: {parsed_data}"
                )
        except Exception as e:

            if rclpy.ok():
                self.get_logger().error(f"Serial read loop terminated with error: {e}")


def main(args=None):
    """
    Main entry point for the RTCM node.

    Initializes the ROS 2 system, creates the RTCM node, and starts spinning the event loop.
    """
    rclpy.init(args=args)
    rtcm_node = RtcmNode()
    rclpy.spin(rtcm_node)
    rtcm_node.destroy_node()
    rclpy.shutdown()