#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_ens160
import board

from interfaces.msg import GasSensorReading


class GasSensor(Node):
    """
    ROS 2 Node that reads environmental data (temperature, humidity, pressure,
    CO2, and TVOC) from the BME280 and ENS160 sensors and publishes it to
    the 'gas_sensor' topic.
    """

    def __init__(self):
        """
        Initializes the GasSensor node, sets up I2C communication, configures
        sensors (BME280 and ENS160), declares parameters, and sets up a timer
        for publishing sensor readings.
        """
        super().__init__("gas_sensor")

        i2c = board.I2C()

        self.declare_parameter("sea_level_pressure_hpa", 1013.25)
        self.declare_parameter("gas_sensor_update_interval_s", 0.2)

        try:
            self.bms280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
            self.ens160 = adafruit_ens160.ENS160(i2c)
        except RuntimeError:
            raise RuntimeError("Gas Sensor Not Connected")

        self.bms280.sea_level_pressure = (
            self.get_parameter("sea_level_pressure_hpa")
            .get_parameter_value()
            .double_value
        )
        self.bms280.mode = adafruit_bme280.MODE_NORMAL

        # self.ens160.reset() # not necessary AFAIK
        self.ens160.mode = adafruit_ens160.MODE_STANDARD

        self.sensor_reading_pub = self.create_publisher(
            GasSensorReading, "gas_sensor", 10
        )
        self.create_timer(
            self.get_parameter("gas_sensor_update_interval_s")
            .get_parameter_value()
            .double_value,
            self.loop,
        )

    def loop(self):
        """
        Periodically reads data from the BME280 and ENS160 sensors, packages
        the readings into a GasSensorReading message, and publishes it if
        there are any subscribers.
        """
        if self.sensor_reading_pub.get_subscription_count() > 0:
            temperature = self.bms280.temperature
            humidity = self.bms280.humidity

            reading = GasSensorReading()
            reading.header.stamp = self.get_clock().now().to_msg()
            reading.temperature_c = temperature
            reading.pressure_pa = self.bms280.pressure * 100  # convert from hPa to Pa
            reading.humidity_rh = humidity

            self.ens160.temperature_compensation = temperature
            self.ens160.humidity_compensation = humidity

            reading.co2_ppm = self.ens160.eCO2
            reading.tvoc_ppb = self.ens160.TVOC

            self.sensor_reading_pub.publish(reading)


def main(args=None):
    """
    Initializes the ROS 2 python library, starts this node, and enters the
    ROS 2 spin loop to process incoming messages and trigger callbacks.
    """
    rclpy.init(args=args)
    node = GasSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
