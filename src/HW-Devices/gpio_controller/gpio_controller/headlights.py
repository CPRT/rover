import errno

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Int8


class HeadlightNode(Node):
    def __init__(self):
        super().__init__("headlight_node")
        self.load_parameters()
        self.load_paths()

        self.export_pwm(self.left_path)
        self.export_pwm(self.right_path)

        frequency = self.get_parameter("frequency").value
        if frequency <= 100:
            self.get_logger().error("Frequency must be greater than 100 Hz")
            frequency = 100

        self.period_ns = int(1e9 / frequency)

        self.disable_pwm(self.left_path)
        self.disable_pwm(self.right_path)

        self.set_period(self.left_path, self.period_ns)
        self.set_period(self.right_path, self.period_ns)

        self.set_duty(self.left_path, 0)
        self.set_duty(self.right_path, 0)

        self.enable_pwm(self.left_path)
        self.enable_pwm(self.right_path)

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.left_subscription = self.create_subscription(
            Int8,
            "left_headlight",
            self.left_callback,
            qos_profile,
        )

        self.right_subscription = self.create_subscription(
            Int8,
            "right_headlight",
            self.right_callback,
            qos_profile,
        )
        self.get_logger().info("Initialization Complete")

    def load_paths(self):
        left_chip_id = self.get_parameter("left_pwm_chip").value
        right_chip_id = self.get_parameter("right_pwm_chip").value
        self.base_path = self.get_parameter("base_class_path").value.rstrip("/")

        self.left_path = f"{self.base_path}/pwmchip{left_chip_id}"
        self.right_path = f"{self.base_path}/pwmchip{right_chip_id}"
    
    def write(self, path, value) -> bool:
        try:
            with open(path, "w", encoding="utf-8") as sys_file:
                sys_file.write(value)
        except PermissionError:
            self.get_logger().error(
                f"Permission denied writing to {path}; run with sufficient privileges."
            )
            return False
        except OSError as error:
            if error.errno != errno.EBUSY:
                self.get_logger().error(f"Failed to write {value} at {path}: {error}")
            return False
        self.get_logger().debug(f"Wrote {value} to {path}")
        return True

    def export_pwm(self, path: str):
        path = path + "/export"
        self.write(path, "0")
        

    def set_period(self, path: str, period_ns: int):
        path = path + "/pwm0/period"
        self.write(path, str(period_ns))

    def set_duty(self, path: str, duty_percent: int):
        if duty_percent > 100:
            self.get_logger().warning(
                f"Duty cannot be greater than 100 ({duty_percent})"
            )
            duty_percent = 100
        elif duty_percent < 0:
            self.get_logger().warning(f"Duty cannot be less than 0 ({duty_percent})")
            duty_percent = 0

        duty_ns = int(self.period_ns * duty_percent / 100)
        path = path + "/pwm0/duty_cycle"
        self.write(path, str(duty_ns))

    def enable_pwm(self, path: str):
        path = path + "/pwm0/enable"
        self.write(path, "1")

    def disable_pwm(self, path: str):
        path = path + "/pwm0/enable"
        self.write(path, "0")

    def left_callback(self, message: Int8):
        self.set_duty(self.left_path, message.data)

    def right_callback(self, message: Int8):
        self.set_duty(self.right_path, message.data)

    def destroy_node(self):
        self.set_duty(self.left_path, 0)
        self.set_duty(self.right_path, 0)
        self.disable_pwm(self.left_path)
        self.disable_pwm(self.right_path)
        super().destroy_node()

    def load_parameters(self):
        self.declare_parameter("left_pwm_chip", 0)
        self.declare_parameter("right_pwm_chip", 2)
        self.declare_parameter("base_class_path", "/sys/class/pwm")
        self.declare_parameter("frequency", 1000)


def main(args=None):
    rclpy.init(args=args)
    node = HeadlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
