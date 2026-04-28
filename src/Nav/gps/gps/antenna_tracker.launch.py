"""
antenna_tracker.launch.py
Launches all four antenna tracker nodes with their default parameters.
Override any value from the command line:
    ros2 launch antenna_tracker antenna_tracker.launch.py SvinMinDur:=120
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # GPS Base Node
        Node(
            package="antenna_tracker",
            executable="gps_base_pub_node",
            name="gps_base_pub_node",
            parameters=[{
                "Device":      "/dev/ttyACM0",   # USB port to u-blox module
                "Baudrate":    38400,
                "Freq":        10.0,              # Hz — how fast to drain serial buffer
                "SvinMinDur":  60,                # seconds of survey-in minimum
                "SvinAccLimit": 10000,            # mm — position accuracy threshold
                "QueueDepth":  10,
            }],
            output="screen",
        ),

        # IMU Node
        # Node(
        #     package="antenna_tracker",
        #     executable="imu_node",
        #     name="imu_node",
        #     parameters=[{
        #         "I2cBus":        1,       # /dev/i2c-1 (RPi default)
        #         "I2cAddr":       0x68,    # ICM-42688 default (AD0 low)
        #         "Freq":          50.0,    # Hz
        #         "MagDeclination": -10.5,  # degrees — update for your location
        #     }],
        #     output="screen",
        # ),

        # Pointing Node
        # Node(
        #     package="antenna_tracker",
        #     executable="pointing_node",
        #     name="pointing_node",
        #     parameters=[{
        #         "Freq":       10.0,   # Hz — calculation rate
        #         "MinRangeM":  5.0,    # metres — ignore rover if closer than this
        #     }],
        #     output="screen",
        # ),

        # RoboClaw Node
        Node(
            package="antenna_tracker",
            executable="antenna_roboclaw_node",
            name="antenna_roboclaw_node",
            parameters=[{
                "Device":      "/dev/ttyS0",  # RPi UART (pins 8/10) or /dev/ttyUSB0
                "Baudrate":    115200,
                "Address":     0x80,          # 128 = default RoboClaw address
                "CountsPerRev": 200_000,      # encoder_ppr × gear_ratio × 4
                "MaxSpeed":    10_000,        # QPPS — max speed for position moves
                "Accel":       5_000,         # QPPS² — acceleration/deceleration
                "EncReadFreq": 10.0,          # Hz — how often to read encoder
            }],
            output="screen",
        ),
    ])
