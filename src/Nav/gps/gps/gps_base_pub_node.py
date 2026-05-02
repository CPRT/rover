import rclpy
import time
from typing import cast
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rtcm_msgs.msg import Message as Rtcm
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from interfaces.msg import SvinStatus
from .ubx_io_manager import UbxIoManager
from pyubx2 import (
    UBXMessage,
    UBX_PROTOCOL,
    RTCM3_PROTOCOL,
    SET,
)


class GpsBaseNode(Node):
    """
    ROS 2 node for managing RTCM data via serial communication.

    1 - Sends UBX config commands to the module to enable Survey-In mode
        and request NAV-SVIN status messages.
    2 - Reads combined UBX and RTCM messages from serial port.
    3 - Routes NAV-SVIN messages to /base_station/svin_status topic so other
        nodes know when the antenna position is locked.
    4 - Routes RTCM correction bytes to /rtcm topic.

    Attributes:
    - fix_pub (Publisher): "/base_station/fix" Publishes tracker fix with
        a depth of 1 (latched) telemetry for web UI and other logging
    - rtcm_pub (Publisher): Publishes RTCM messages to the /rtcm topic.
    - valid_pub (Publisher): Publishes true when survey-in is done (latched)
    - svin_status_pub (Publisher): Publishes duration and mean accuracy of survey-in process for monitoring
    - serial_conn (IoManager): Manages serial I/O operations.
    - layers (int): Configuration layers for the UBXMessage.
    - timer (Timer): Timer for periodically reading and publishing RTCM data.
    """

    def __init__(self):
        super().__init__("gps_base_node")
        self.load_params()
        self._setup_publishers()

        self.serial_conn = UbxIoManager(
            port=self.dev,
            baud=self.baudrate,
            msg_filter=UBX_PROTOCOL | RTCM3_PROTOCOL,
        )
        self._configure_module()
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)
        self.get_logger().info("GPS base node started.")

    def load_params(self):
        """
        Loads parameters from the ROS 2 parameter server with default values.
        Parameters include timing mode, survey-in settings, persistence, frequency,
        baud rate, and serial device.
        """
        self.declare_parameter("Freq", 0.25)
        self.freq = self.get_parameter("Freq").get_parameter_value().double_value
        if self.freq <= 0:
            self.get_logger().warn("Frequency must be positive. Defaulting to 2.0 Hz.")
            self.freq = 2.0

        self.declare_parameter("Baudrate", 38400)
        self.baudrate = (
            self.get_parameter("Baudrate").get_parameter_value().integer_value
        )

        self.declare_parameter(
            "Device",
            "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00",
        )
        self.dev = self.get_parameter("Device").get_parameter_value().string_value

        self.declare_parameter("QueueDepth", 10)
        self.queue_depth = (
            self.get_parameter("QueueDepth").get_parameter_value().integer_value
        )

        self.declare_parameter("SvinMinDur", 60)
        self.svin_min_dur = (
            self.get_parameter("SvinMinDur").get_parameter_value().integer_value
        )

        self.declare_parameter("SvinAccLimit", 10_000)  # 10 000 mm
        self.svin_acc_limit = (
            self.get_parameter("SvinAccLimit").get_parameter_value().integer_value
        )
        self.get_logger().info(
            f"Min duration: {self.svin_min_dur}s, Min acc {self.svin_acc_limit}mm"
        )

    def _setup_publishers(self):
        # keep depth at one for RTCM and use the new default for svin msgs
        self.rtcm_pub = self.create_publisher(Rtcm, "/rtcm", 1)

        self.fix_pub = self.create_publisher(
            NavSatFix, "/base_station/fix", self.queue_depth
        )
        self.svin_status_pub = self.create_publisher(
            SvinStatus, "/base_station/svin_status", self.queue_depth
        )

    def _configure_module(self):
        """
        Sends two UBX config messages to the GPS module:

        CFG-TMODE3 puts the receiver into Survey-In mode with given params.
        The module will average its position and once both thresholds are met,
        declare its position valid and begin outputting RTCM corrections.

        CFG-MSG: tells the module to emit NAV-SVIN status messages once per
        second on the current serial port, so we can monitor progress.
        """
        try:
            # Send 2 messages with different svinMinDur to restart survey_in
            msg = UBXMessage(
                "CFG",
                "CFG-TMODE3",
                SET,
                version=0,
                rcvrMode=1,
                svinMinDur=1,
                svinAccLimit=self.svin_acc_limit * 10,  # conv 1mm -> 0.1mm
            )
            self.serial_conn.write(msg.serialize())
            time.sleep(1)
            msg = UBXMessage(
                "CFG",
                "CFG-TMODE3",
                SET,
                rcvrMode=1,
                svinMinDur=self.svin_min_dur,
                svinAccLimit=self.svin_acc_limit * 10,  # conv 1mm -> 0.1mm
            )
            self.serial_conn.write(msg.serialize())

            # Enable NAV-SVIN output on this port
            # CFG-MSG enables/disables individual message types per port.
            # rateI2C/rateUART1/.../rateUSB are output rates (messages per epoch).
            # We set rateUSB=1 (once per navigation epoch, typically 1 Hz).
            cfg_msg = UBXMessage(
                "CFG",
                "CFG-MSG",
                SET,
                # msgClass=UBX_NAV_CLASS,
                # msgID=UBX_NAV_SVIN_ID,
                rateI2C=0,
                rateUART1=0,
                rateUART2=0,
                rateUSB=1,  # output NAV-SVIN once per epoch on USB
                rateSPI=0,
            )
            self.serial_conn.write(cfg_msg.serialize())
            self.get_logger().info("Enabled NAV-SVIN output on USB port.")

        except Exception as e:
            self.get_logger().error(f"Failed to configure GPS module: {e}")

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

                if not raw or parsed_data is None:
                    # for read timeouts or empty reads, just continue

                    if rclpy.ok():
                        self.get_logger().info("Serial read returned no data.")

                        # short pause to avoid busy loop if read()
                        # this should rarely happen
                        rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                raw = cast(bytes, raw)
                identity = getattr(parsed_data, "identity", "")

                if identity == "NAV-SVIN":
                    self._handle_svin(parsed_data)
                elif raw[0:2] != b"\xb5b":  # pylint: disable=unsubscriptable-object
                    # Not a UBX sync header → it's RTCM
                    self._handle_rtcm(raw)

        except Exception as e:

            if rclpy.ok():
                self.get_logger().error(f"Serial read loop terminated with error: {e}")

    def _handle_rtcm(self, raw: bytes):
        """Wrap raw RTCM bytes in a ROS message and publish."""
        msg = Rtcm()
        msg.message = list(raw)
        self.rtcm_pub.publish(msg)

    def _handle_svin(self, parsed):
        """
        Extract Survey-In status fields and publish to monitoring topics.

        Key fields on a NAV-SVIN message:
            dur     - elapsed survey-in time in seconds
            meanAcc - current mean position accuracy in mm (×0.1 for u-blox raw)
            valid   - True when survey-in has finished successfully
            meanX/Y/Z - ECEF position in cm (we convert to lat/lon/alt via WGS84)
        """
        dur = getattr(parsed, "dur", 0)
        acc_mm = (
            getattr(parsed, "meanAcc", 0) * 0.1
        )  # meanAcc from pyubx2 is in 0.1mm units → convert to mm
        valid = bool(getattr(parsed, "valid", False))
        active = bool(getattr(parsed, "active", False))

        # Publish raw status fields for monitoring/dashboards
        status = SvinStatus()
        status.dur_sec = int(dur)
        status.acc_mm = float(acc_mm)
        self.svin_status_pub.publish(status)

        self.get_logger().info(
            f"Survey-In: dur={dur}s  acc={acc_mm:.1f}mm  "
            f"valid={valid}  active={active}"
        )

        self._publish_fix_from_ecef(parsed, acc_mm)

    def _publish_fix_from_ecef(self, parsed, mean_acc):
        """
        Convert Earth-Centred Earth-Fixed (ECEF) coordinates from NAV-SVIN
        into geodetic lat/lon/alt (WGS84) and publish as NavSatFix.

        NAV-SVIN gives meanX/Y/Z in cm. Converts to metres first.
        """
        import math

        # NAV-SVIN ECEF values are in cm
        x = getattr(parsed, "meanX", 0) * 0.01  # cm to m
        y = getattr(parsed, "meanY", 0) * 0.01
        z = getattr(parsed, "meanZ", 0) * 0.01

        # WGS84 ellipsoid constants
        a = 6_378_137.0  # semi-major axis (m)
        e2 = 6.6943799901414e-3  # first eccentricity squared

        lon = math.atan2(y, x)
        p = math.sqrt(x ** 2 + y ** 2)
        lat = math.atan2(z, p * (1 - e2))  # initial estimate

        # Iterate to convergence (usually 3–4 iterations)
        for _ in range(10):
            sin_lat = math.sin(lat)
            N = a / math.sqrt(1 - e2 * sin_lat ** 2)
            lat_new = math.atan2(z + e2 * N * sin_lat, p)
            if abs(lat_new - lat) < 1e-12:
                break
            lat = lat_new

        sin_lat = math.sin(lat)
        N = a / math.sqrt(1 - e2 * sin_lat ** 2)
        alt = (
            p / math.cos(lat) - N
            if abs(math.cos(lat)) > 1e-10
            else abs(z) / abs(sin_lat) - N * (1 - e2)
        )

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "base_antenna"
        fix.latitude = math.degrees(lat)
        fix.longitude = math.degrees(lon)
        fix.altitude = alt
        fix.status.status = (
            2 if self.svin_acc_limit > mean_acc else -2
        )  # STATUS_GBAS_FIX (RTK)
        self.fix_pub.publish(fix)

        self.get_logger().info(
            f"Base station fix:  status={fix.status.status} "
            f"lat={fix.latitude:.8f} lon={fix.longitude:.8f}  alt={fix.altitude:.3f}m"
        )


def main(args=None):
    """
    Main entry point for the RTCM node.

    Initializes the ROS 2 system, creates the RTCM node, and starts spinning the event loop.
    """
    rclpy.init(args=args)
    rtcm_node = GpsBaseNode()
    rclpy.spin(rtcm_node)
    rtcm_node.destroy_node()
    rclpy.shutdown()
