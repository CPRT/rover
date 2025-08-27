# Servo Controller
Control multiple servos using motor controllers with ROS2 and Python.

## Description
This project provides a Python-based interface to control servos using the Pololu Maestro controller. 

## Requirements
### Software
- Python 3.6 or higher
- Pololu Maestro Control Center (optional for configuration)

### Python Dependencies
- `pyserial`
- `rclpy` (for ROS2 integration)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/CPRT/cprt_rover_24.git

2. Install Python dependencies:
    ```bash
    pip3 install pyserial
    pip3 install adafruit-circuitpython-lis3dh
    pip3 install adafruit-circuitpython-pca9685
    pip3 install adafruit-circuitpython-motorkit 

## How to Use
1. Setup:
    ```bash 
    colcon build
    source install/setup.bash

2. Starting the USB service
    ```bash
    ros2 run servo_pkg usb_servo_service

3. OR Starting the i2c service
    ```bash
    ros2 run servo_pkg i2c_servo_service

4. Starting the client
    The servo_client.py contains some sample code that controls the servo with the USB or the i2c controllers

    If one of the controllers is not connected don't try to use the method in main, it will make the other not work.
    ```bash
    ros2 run servo_pkg servo_client 
