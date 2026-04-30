import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Bool


class AntennaPointingNode(Node):
    def __init__(self):
        super().__init__("antenna_pointing_node")

        # Parameters
        self.declare_parameter("Freq", 10)
        self.freq = self.get_parameter("Freq").value

        # State and fix values
        self.base_fix = None
        self.rover_fix = None
        self.svin_valid = False

        self.zero_offset = 0
        self.calibrated = False

        # ROS things
        self.create_subscription(NavSatFix, "/base_station/fix", self.base_cb, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self.rover_cb, 10)
        self.create_subscription(Bool, "/base_station/svin_valid", self.svin_cb, 10)

        self.bearing_pub = self.create_publisher(
            Float32, "/antenna/tracker_bearing", 10
        )

        self.timer = self.create_timer(1.0 / self.freq, self.update)

    def base_cb(self, msg):
        self.base_fix = msg

    def rover_cb(self, msg):
        self.rover_fix = msg

    def svin_cb(self, msg):
        self.svin_valid = msg

    def update(self):
        if not self.svin_valid or not self.base_fix or not self.rover_fix:
            return

        # ----- HANDLE IMU HERE

        # ----- HANDLE CALIBRATION HERE (maybe)

        bearing = self.bearing(
            self.base_fix.latitude,
            self.base_fix.longitude,
            self.rover_fix.latitude,
            self.rover_fix.longitude,
        )

        self.publish_bearing(bearing)

    def publish_bearing(self, bearing):
        msg = Float32()
        msg.data = float(bearing)
        self.bearing_pub.publish(msg)

    def bearing(self, lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon1 - lon2)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        return (math.atan2(x, y) + math.pi * 2) % (math.pi * 2)


def main(args=None):
    rclpy.init(args=args)
    node = AntennaPointingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
