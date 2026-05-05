import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from std_msgs.msg import UInt8MultiArray, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
import struct
import zlib


class GridMapDecompressor(Node):
    PROTOCOL_VERSION = 0x01

    def __init__(self):
        super().__init__("gridmap_decompressor")

        radio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        rviz_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(GridMap, "/viz/traversability_map", rviz_qos)

        self.sub = self.create_subscription(
            UInt8MultiArray,
            "/telemetry/gridmap_compressed",
            self.map_callback,
            radio_qos,
        )

    def map_callback(self, msg):
        try:
            # 1. Unzip bytes
            compressed_bytes = bytes(msg.data)
            raw_payload = zlib.decompress(compressed_bytes)

            # 2. Validate minimum header size before unpacking.
            struct_size = struct.calcsize("<2B 10d 2H")
            if len(raw_payload) < struct_size:
                self.get_logger().warning(
                    "Dropped gridmap packet: payload smaller than header."
                )
                return

            # 3. Extract metadata and verify protocol version.
            meta = struct.unpack("<2B 10d 2H", raw_payload[:struct_size])
            protocol_version = meta[0]
            if protocol_version != self.PROTOCOL_VERSION:
                self.get_logger().warning(
                    f"Dropped gridmap packet: protocol version mismatch "
                    f"({protocol_version} != {self.PROTOCOL_VERSION})."
                )
                return

            frame_len = meta[1]
            resolution, length_x, length_y = meta[2], meta[3], meta[4]
            px, py, pz, qx, qy, qz, qw = (
                meta[5],
                meta[6],
                meta[7],
                meta[8],
                meta[9],
                meta[10],
                meta[11],
            )
            outer_start, inner_start = meta[12], meta[13]

            # 4. Extract Frame ID string
            frame_start = struct_size
            frame_end = struct_size + frame_len
            if len(raw_payload) < frame_end:
                self.get_logger().warning(
                    "Dropped gridmap packet: truncated frame_id bytes."
                )
                return
            frame_id = raw_payload[frame_start:frame_end].decode("utf-8")

            # 5. Validate alignment and split remaining bytes.
            # Every cell has 2 bytes for int16 elevation, 1 byte for uint8 traversability = 3 bytes total
            remaining_bytes = raw_payload[frame_end:]
            if len(remaining_bytes) % 3 != 0:
                self.get_logger().warning(
                    "Dropped gridmap packet: payload alignment is invalid."
                )
                return
            num_cells = len(remaining_bytes) // 3

            elev_bytes = remaining_bytes[: num_cells * 2]
            trav_bytes = remaining_bytes[num_cells * 2 :]

            # 6. Load back into Numpy
            # Elevation samples are encoded on-wire as little-endian int16.
            elev_int16 = np.frombuffer(elev_bytes, dtype=np.dtype("<i2"))
            trav_uint8 = np.frombuffer(trav_bytes, dtype=np.uint8)

            # 7. Reconstruct Floats and restore NaNs
            elev_float = np.where(
                elev_int16 == -32768, np.nan, elev_int16 / 1000.0
            ).astype(np.float32)
            trav_float = np.where(
                trav_uint8 == 255, np.nan, trav_uint8.astype(np.float32) / 100.0
            )

            # 8. Build the fresh ROS 2 GridMap message
            grid_msg = GridMap()
            grid_msg.header.stamp = self.get_clock().now().to_msg()  # Fix desync
            grid_msg.header.frame_id = frame_id

            grid_msg.info.resolution = resolution
            grid_msg.info.length_x = length_x
            grid_msg.info.length_y = length_y
            grid_msg.info.pose.position.x = px
            grid_msg.info.pose.position.y = py
            grid_msg.info.pose.position.z = pz
            grid_msg.info.pose.orientation.x = qx
            grid_msg.info.pose.orientation.y = qy
            grid_msg.info.pose.orientation.z = qz
            grid_msg.info.pose.orientation.w = qw

            grid_msg.layers = ["elevation", "traversability"]
            grid_msg.basic_layers = []  # Usually empty for basic visualization
            grid_msg.outer_start_index = outer_start
            grid_msg.inner_start_index = inner_start

            # Package arrays into Float32MultiArray wrappers
            elev_multi = Float32MultiArray()
            elev_multi.data = elev_float.tolist()

            trav_multi = Float32MultiArray()
            trav_multi.data = trav_float.tolist()

            grid_msg.data = [elev_multi, trav_multi]

            # 9. Render in RViz!
            self.pub.publish(grid_msg)

        except Exception as e:
            self.get_logger().error(f"Decompression failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GridMapDecompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
