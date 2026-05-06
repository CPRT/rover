import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from smbus2 import SMBus

I2C_DEFAULT_ADDR = 0x21

TOGGLE_REG = 0x08
PG_REG = 0x09

class PDB_Rails(Node):
    def __init__(self):
        super().__init__("pdb_rails")

        self.declare_parameter("i2c_bus", 7)  # /dev/i2c-1 on most SBCs
        self.declare_parameter("i2c_address", I2C_DEFAULT_ADDR)
        self.declare_parameter("poll_freq", 4)

        self.i2c_bus = self.get_parameter("i2c_bus").get_parameter_value().integer_value
        self.i2c_addr = (
            self.get_parameter("i2c_address").get_parameter_value().integer_value
        )
        self.poll_freq = self.get_parameter("poll_freq").get_parameter_value().integer_value

        self.bus = SMBus(self.i2c_bus)
        
        self.pg_pub = self.create_publisher(UInt8, "~/pdb_pg", 3)
        self.toggle_sub = self.create_subscription(UInt8, "~/pdb_toggle", self.toggle, 3)

        self.create_timer(1 / self.poll_freq, self.loop)
        self.get_logger().info("PDB rail monitoring node started")

    def loop(self):
        pg_reg = self.bus.read_byte_data(self.i2c_addr, PG_REG)

        msg = UInt8()
        msg.data = pg_reg

        self.pg_pub.publish(msg)

    def toggle(self, msg):
        self.bus.write_i2c_block_data(self.i2c_addr, TOGGLE_REG, msg.data)

    def destroy_node(self):
        try:
            self.bus.close()
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PDB_Rails()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
