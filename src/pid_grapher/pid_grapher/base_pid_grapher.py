import threading
from ros_phoenix.msg import MotorStatus
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time as time
import matplotlib

matplotlib.use("TkAgg")  # make sure we have an interactive backend

import matplotlib.pyplot as plt
from sensor_msgs.msg import Joy
from rclpy.executors import MultiThreadedExecutor

"""
For running this, there are some things you may want to note:
    - You have to start the Joy node with ros2 run joy joy_node
    - This is specifically for the base motor
    - If you close the window, the program stops because python lkw just like that
"""


class BasePIDGrapher(Node):
    def __init__(self):
        self.joystickVelocity = 0.0
        super().__init__("base_pid_grapher")
        self.subscription = self.create_subscription(
            MotorStatus,
            "/base/status",
            self.plotter_callback,
            10,
        )
        self.subscription = self.create_subscription(
            Joy, "/joy", self.joystick_callback, 10
        )
        print("TEST - Maybe found callback? Also running node")
        self.targetVelocity = 10

        plt.ion()

        plt.rcParams["toolbar"] = "none"

        self.fig, self.ax = plt.subplots()
        self.x_data, self.y_data = [], []
        self.x_data2, self.y_data2 = [], []

        (self.line1,) = self.ax.plot(self.x_data, self.y_data, "b-")
        (self.line2,) = self.ax.plot(self.x_data2, self.y_data2, "b-")

        self.fig.show()

        self.subscription

        self.beginning = self.get_clock().now()

    def plotGraph(self, x, y, setpoint):
        print("Graphing")
        self.x_data.append(x)
        self.y_data.append(y)
        self.x_data2.append(x)
        self.y_data2.append(setpoint)

        self.line1.set_xdata(self.x_data)
        self.line1.set_ydata(self.y_data)
        self.line2.set_xdata(self.x_data2)
        self.line2.set_ydata(self.y_data2)

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        plt.pause(0.001)

    def plotter_callback(self, msg):
        self.get_logger().info(f"Velocity recieved: {msg.velocity}")
        now = self.get_clock().now()

        print("There is something running in the callback")
        delta = now - self.beginning
        x = delta.nanoseconds / 1e9

        self.plotGraph(x, float(msg.velocity), self.targetVelocity)

    def joystick_callback(self, stick):
        self.joystickVelocity = stick.axes[1]
        print(self.joystickVelocity)

    def _show_plot(self):
        plt.show(block=True)


def main(args=None):
    print("TEST - Running Main")
    rclpy.init(args=args)
    node = BasePIDGrapher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
