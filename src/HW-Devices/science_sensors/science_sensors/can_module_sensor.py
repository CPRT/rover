#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import can
import struct


class CANListener(can.Listener):
    def __init__(self, process_callback):
        super().__init__()
        self.process_callback = process_callback

    def on_message_received(self, msg):
        self.process_callback(msg)


class ScienceReader(Node):
    """
    ROS 2 Node that reads sensor data from CAN bus messages.
    Assumes each CAN message encodes a float64 (8 bytes).
    """

    def __init__(self):
        super().__init__("science_reader")

        self.declare_parameter("sensors", ["ozone", "hydrogen"])
        self.sensors = self.get_parameter("sensors").value

        self.sensor_dict = {}
        filters = []
        default_id = 100

        for sensor in self.sensors:
            self.declare_parameter(f"{sensor}.msg_id", default_id)
            msg_id = (
                self.get_parameter(f"{sensor}.msg_id")
                .get_parameter_value()
                .integer_value
            )
            if msg_id in self.sensor_dict:
                self.get_logger().warn("Duplicate msg ID")
            filters.append({"can_id": msg_id, "can_mask": 0x7FF})
            self.sensor_dict[msg_id] = self.create_publisher(Float64, sensor, 10)
            default_id += 1

        # Set up CAN bus with filters
        self.bus = can.interface.Bus(
            channel="can0", bustype="socketcan", can_filters=filters
        )

        # Set up a listener with a notifier
        self.listener = CANListener(self.process_message)
        self.notifier = can.Notifier(self.bus, [self.listener])

    def process_message(self, msg):
        msg_id = msg.arbitration_id
        if msg_id not in self.sensor_dict:
            self.get_logger().warn(f"Unknown CAN ID received: 0x{msg_id:X}")
            return

        # Expecting 8 bytes representing a float64
        if len(msg.data) != 8:
            self.get_logger().error(
                f"Invalid data length from CAN ID 0x{msg_id:X}: {len(msg.data)}"
            )
            return

        try:
            value = struct.unpack("<d", msg.data)[0]
            ros_msg = Float64()
            ros_msg.data = value
            self.get_logger().info(f"Received data from CAN ID 0x{msg_id:X}: {value}")
            self.sensor_dict[msg_id].publish(ros_msg)
        except struct.error as e:
            self.get_logger().error(
                f"Failed to unpack float from CAN ID 0x{msg_id:X}: {e}"
            )

    def destroy_node(self):
        self.notifier.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScienceReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
