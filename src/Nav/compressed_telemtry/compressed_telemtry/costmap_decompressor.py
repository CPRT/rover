import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import UInt8MultiArray
from rclpy.serialization import deserialize_message
import zlib

class CostmapDecompressor(Node):
    def __init__(self):
        super().__init__('costmap_decompressor')

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
        
        # --- Publishers (To Local RViz) ---
        self.full_map_pub = self.create_publisher(
            OccupancyGrid, '/viz/global_costmap/costmap', rviz_qos)
        self.update_pub = self.create_publisher(
            OccupancyGridUpdate, '/viz/global_costmap/costmap_updates', 10)

        # --- Subscribers (From Radio Link) ---
        self.full_map_sub = self.create_subscription(
            UInt8MultiArray, '/telemetry/costmap_full_compressed', self.full_map_callback, radio_qos)
        self.update_sub = self.create_subscription(
            UInt8MultiArray, '/telemetry/costmap_updates_compressed', self.update_callback, radio_qos)

        self.get_logger().info("Costmap Decompressor Node Started.")

    def full_map_callback(self, msg):
        try:
            compressed_bytes = bytes(msg.data)
            raw_bytes = zlib.decompress(compressed_bytes)
            
            # Rebuild original ROS 2 message
            grid_msg = deserialize_message(raw_bytes, OccupancyGrid)
            
            # Cheat the timestamp to current time to prevent TF "Extrapolation into past" errors
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.full_map_pub.publish(grid_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to unpack full map: {e}")

    def update_callback(self, msg):
        try:
            compressed_bytes = bytes(msg.data)
            raw_bytes = zlib.decompress(compressed_bytes)
            
            # Rebuild original ROS 2 update message
            update_msg = deserialize_message(raw_bytes, OccupancyGridUpdate)
            
            # Cheat the timestamp on the update as well
            update_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.update_pub.publish(update_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to unpack map update: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CostmapDecompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()