import rclpy
from std_msgs.msg import Float32
from smbus2 import SMBus

from servo_pkg.parent_config import Parent_Config

I2C_DEFAULT_ADDR = 0x21

HW_NUM_SERVOS = 4

REG_SERVO1_HIGH = 0x00


def convert_from_radians(angle, servo_info):
    total_range = servo_info.max - servo_info.min
    return int(servo_info.min + (total_range * angle / servo_info.rom))


def convert_to_radians(value, servo_info):
    total_range = servo_info.max - servo_info.min
    return servo_info.rom * (value - servo_info.min) / total_range


class PDB_Servo(Parent_Config):
    def __init__(self):
        super().__init__("pdb_servo", HW_NUM_SERVOS)

        # I2C parameters
        self.declare_parameter("i2c_bus", 7)  # /dev/i2c-1 on most SBCs
        self.declare_parameter("i2c_address", I2C_DEFAULT_ADDR)

        self.i2c_bus = self.get_parameter("i2c_bus").get_parameter_value().integer_value
        self.i2c_addr = (
            self.get_parameter("i2c_address").get_parameter_value().integer_value
        )

        self.bus = SMBus(self.i2c_bus)

        self.subs = {}
        for port in range(HW_NUM_SERVOS):
            self.subs[port] = self.create_subscription(
                Float32,
                f"~/{self.servo_info[port].motor_name}",
                lambda msg, port=port: self.set_position(msg, port),
                3,
            )
            self.get_logger().info(f"Topic: ~/{self.servo_info[port].motor_name}")
        self._last_pwm_us = {p: None for p in range(HW_NUM_SERVOS)}

    def _write_servo_us(self, port: int, pwm_us: int):
        base_reg = REG_SERVO1_HIGH + (2 * port)
        high = (pwm_us >> 8) & 0xFF
        low = pwm_us & 0xFF
        self.bus.write_i2c_block_data(self.i2c_addr, base_reg, [high, low])

    def set_position(self, msg, port):
        self.check_valid_servo(port)
        servo_info = self.servo_info[port]

        target_value_us = convert_from_radians(msg.data, servo_info)
        current_position = (
            None
            if self._last_pwm_us[port] is None
            else convert_to_radians(self._last_pwm_us[port], servo_info)
        )

        if not (servo_info.min <= target_value_us <= servo_info.max):
            self.get_logger().warning(
                f"Servo {port} input out of range. " f"Last angle: {current_position}"
            )
            return

        self._write_servo_us(port, target_value_us)
        self._last_pwm_us[port] = target_value_us

        new_angle = convert_to_radians(target_value_us, servo_info)
        self.get_logger().info(
            f"Servo {port} -> angle: {new_angle} rad, PWM: {target_value_us} us"
        )

    def destroy_node(self):
        try:
            self.bus.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PDB_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
