Requires external DHT library (DHT sensor library by Adafruit in the ArduinoIDE library browser)
Compiled using ArduinoIDE.
Sensor pins are hard-coded to:

METHANE_PIN     = 4;
CO2_PIN         = 12;
POLARIMETER_PIN = 14;
DHT_PIN = 2;

Run the esp_serial_bridge ROS node to connect the esp to the ROS environment through matching topics.