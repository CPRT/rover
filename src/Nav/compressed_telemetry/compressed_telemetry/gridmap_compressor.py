import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from std_msgs.msg import UInt8MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
import struct
import zlib


class GridMapCompressor(Node):
    PROTOCOL_VERSION = 0x01

    def __init__(self):
        super().__init__("gridmap_compressor")

        self.declare_parameter("log_compression_stats", True)
        self.log_compression_stats = self.get_parameter("log_compression_stats").value

        # Stats window for bandwidth/ratio measurement over 10s
        self.declare_parameter("log_mbps_stats", True)
        self.log_mbps_stats = self.get_parameter("log_mbps_stats").value

        self._raw_bytes_window = 0
        self._compressed_bytes_window = 0
        self._packets_window = 0
        # Timer to report mbps and average compression every 10 seconds
        self._stats_timer = self.create_timer(10.0, self._stats_timer_callback)

        # QoS for Radio: Aggressive UDP
        radio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        nav2_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(
            UInt8MultiArray, "/telemetry/gridmap_compressed", radio_qos
        )

        self.sub = self.create_subscription(
            GridMap, "/traversability_map", self.map_callback, nav2_qos
        )

    def map_callback(self, msg):
        try:
            # 1. Find the required layers
            if "elevation" not in msg.layers or "traversability" not in msg.layers:
                return

            elev_idx = msg.layers.index("elevation")
            trav_idx = msg.layers.index("traversability")

            # 2. Extract arrays into fast Numpy vectors
            elev_arr = np.array(msg.data[elev_idx].data, dtype=np.float32)
            trav_arr = np.array(msg.data[trav_idx].data, dtype=np.float32)

            # 3. Cast Elevation to int16 (Scale by 1000 for mm precision, -32768 for NaN)
            elev_valid = ~np.isnan(elev_arr)
            elev_int16 = np.full(elev_arr.shape, -32768, dtype=np.int16)
            elev_int16[elev_valid] = np.clip(
                elev_arr[elev_valid] * 1000.0, -32767, 32767
            ).astype(np.int16)

            # 4. Cast Traversability to uint8 (Clip 0-254, 255 for NaN)
            trav_valid = ~np.isnan(trav_arr)
            trav_uint8 = np.full(trav_arr.shape, 255, dtype=np.uint8)
            trav_uint8[trav_valid] = np.clip(
                trav_arr[trav_valid] * 100.0, 0, 254
            ).astype(np.uint8)

            # 5. Pack the vital GridMapInfo metadata
            frame_bytes = msg.header.frame_id.encode("utf-8")

            # Struct Format: B=uint8, d=float64, H=uint16
            # [protocol_version, frame_len, 10 doubles, 2 uint16]
            metadata = struct.pack(
                "<2B 10d 2H",
                self.PROTOCOL_VERSION,
                len(frame_bytes),
                msg.info.resolution,
                msg.info.length_x,
                msg.info.length_y,
                msg.info.pose.position.x,
                msg.info.pose.position.y,
                msg.info.pose.position.z,
                msg.info.pose.orientation.x,
                msg.info.pose.orientation.y,
                msg.info.pose.orientation.z,
                msg.info.pose.orientation.w,
                msg.outer_start_index,
                msg.inner_start_index,
            )

            # 6. Assemble the binary payload and Compress
            # Ensure elevation payload is serialized as little-endian int16 on-wire.
            elev_int16_le = elev_int16.astype(np.dtype("<i2"))
            raw_payload = (
                metadata + frame_bytes + elev_int16_le.tobytes() + trav_uint8.tobytes()
            )
            compressed_bytes = zlib.compress(raw_payload)

            # 7. Publish over UDP
            out_msg = UInt8MultiArray()
            out_msg.data = list(compressed_bytes)
            self.pub.publish(out_msg)

            # Track bytes sent for stats window and optionally log compression stats
            try:
                compressed_len = len(compressed_bytes)
                raw_len = len(raw_payload)
                self._compressed_bytes_window += compressed_len
                self._raw_bytes_window += raw_len
                self._packets_window += 1
                if self.log_compression_stats:
                    self._log_compression_stats("gridmap", raw_len, compressed_len)
            except Exception:
                # defensive: never crash stats collection
                pass

        except Exception as e:
            self.get_logger().error(f"Compression failed: {e}")

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

    def _stats_timer_callback(self):
        # Report compressed network send rate over the last 10 seconds
        if not getattr(self, "log_mbps_stats", False):
            return
        duration_s = 10.0
        compressed = float(self._compressed_bytes_window)
        raw = float(self._raw_bytes_window) if self._raw_bytes_window > 0 else 0.0
        mbps = (compressed * 8.0) / (1e6 * duration_s)  # megabits per second
        if raw > 0.0:
            avg_ratio = compressed / raw
            avg_reduction = (1.0 - avg_ratio) * 100.0
            self.get_logger().info(
                f"Compressed send rate: {mbps:.3f} Mbps over last {int(duration_s)}s "
                f"({int(compressed)} bytes, avg reduction {avg_reduction:.1f}% , avg ratio {avg_ratio:.2f})"
            )
        else:
            self.get_logger().info(
                f"Compressed send rate: {mbps:.3f} Mbps over last {int(duration_s)}s ({int(compressed)} bytes)"
            )
        self._raw_bytes_window = 0
        self._compressed_bytes_window = 0
        self._packets_window = 0


def main(args=None):
    rclpy.init(args=args)
    node = GridMapCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
