import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import UInt8MultiArray
from rclpy.serialization import serialize_message
import zlib


class CostmapCompressor(Node):
    def __init__(self):
        super().__init__("costmap_compressor")

        self.declare_parameter("log_compression_stats", True)
        self.log_compression_stats = self.get_parameter("log_compression_stats").value

        radio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        nav2_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers (To Radio Link) ---
        self.full_map_pub = self.create_publisher(
            UInt8MultiArray, "/telemetry/costmap_full_compressed", radio_qos
        )
        self.update_pub = self.create_publisher(
            UInt8MultiArray, "/telemetry/costmap_updates_compressed", radio_qos
        )

        # --- Subscribers (From Local Nav2) ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.map_callback, nav2_qos
        )
        self.update_sub = self.create_subscription(
            OccupancyGridUpdate,
            "/global_costmap/costmap_updates",
            self.update_callback,
            10,
        )

        # --- State & Timers ---
        self.latest_full_map = None
        # Heartbeat timer: Send a full map refresh every 10 seconds (0.1 Hz)
        self.heartbeat_timer = self.create_timer(10.0, self.heartbeat_callback)

        self.get_logger().info("Costmap Compressor Node Started.")

    def update_callback(self, msg):
        """Instantly compress and forward high-frequency map updates."""
        try:
            raw_bytes = serialize_message(msg)
            compressed_bytes = zlib.compress(raw_bytes)

            if self.log_compression_stats:
                self._log_compression_stats(
                    "update", len(raw_bytes), len(compressed_bytes)
                )

            out_msg = UInt8MultiArray()
            out_msg.data = list(compressed_bytes)
            self.update_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to compress update: {e}")

    def map_callback(self, msg):
        """Cache the latest full map silently in the background."""
        self.latest_full_map = msg

    def heartbeat_callback(self):
        """Once every 10 seconds, send the cached full map to fix any UDP desync."""
        if self.latest_full_map is not None:
            try:
                raw_bytes = serialize_message(self.latest_full_map)
                compressed_bytes = zlib.compress(raw_bytes)

                if self.log_compression_stats:
                    self._log_compression_stats(
                        "full costmap", len(raw_bytes), len(compressed_bytes)
                    )

                out_msg = UInt8MultiArray()
                out_msg.data = list(compressed_bytes)
                self.full_map_pub.publish(out_msg)
                self.get_logger().debug("Sent full map heartbeat.")
            except Exception as e:
                self.get_logger().error(f"Failed to compress full map: {e}")

    def _log_compression_stats(self, label, raw_size, compressed_size):
        if raw_size == 0:
            self.get_logger().info(
                f"Compressed {label}: 0 bytes -> {compressed_size} bytes"
            )
            return

        ratio = compressed_size / raw_size
        reduction = (1.0 - ratio) * 100.0
        self.get_logger().info(
            f"Compressed {label}: {raw_size} bytes -> {compressed_size} bytes "
            f"({reduction:.1f}% smaller, {ratio:.2f}x of original)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CostmapCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
