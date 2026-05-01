import rclpy
from rclpy.node import Node
from interfaces.msg import Distance
import serial

class TimeOfFlightSensor(Node):
    def __init__(self):
        super().__init__("time_of_flight_sensor")
        self.ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_20:6E:F1:69:EE:E0-if00', 115200)
        self.pub = self.create_publisher(Distance, "eef_distance", 10)
        self.create_timer(0.001, self.loop)
        self.get_logger().info("Time of Flight Sensor node started")

    def loop(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            try:
                reading = int(line)
                msg = Distance()
                msg.header.stamp = self.get_clock().now().to_msg()
                if (reading == -2):
                    mm = 0
                    msg.status = Distance.STATUS_ERROR
                elif (reading == -1):
                    mm = 0
                    msg.status = Distance.STATUS_INVALID
                else:
                    msg.status = Distance.STATUS_OK
                    mm = reading
                    msg.distance = mm / 1000.0
                    self.get_logger().debug(f"Distance: {msg.distance:.3f} m")
                self.pub.publish(msg)
            except ValueError:
                pass

rclpy.init()
node = TimeOfFlightSensor()
rclpy.spin(node)