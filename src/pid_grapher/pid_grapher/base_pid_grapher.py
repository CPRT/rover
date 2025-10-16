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
from sensor_msgs.msg import JointState
"""
For running this, there are some things you may want to note:
    - You have to start the Joy node with ros2 run joy joy_node
    - This is specifically for the base motor
    - If you close the window, the program stops because python lkw just like that
"""
# NOTE PLEASE REFACTOR WITH WHAT CONNOR SAID (YOU ARE GOING TO GET LISTS AND STUFF)


class BasePIDGrapher(Node):
    initiated = False
    def __init__(self):
        self.joystickVelocity = 0.0
        super().__init__("base_pid_grapher")
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
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

        self.names = []
        self.figs = []
        self.axs = []
        self.velocities = [[],[],[],[],[],[]]
        self.times = [[],[],[],[],[],[]]
        self.targetVelocities = [[],[],[],[],[],[]]
        self.line1s = []#TODO make this actually work im too fucking lazy right now
        self.line2s = []
        
        for i in range(0, len(self.times)):
            fig, ax = plt.subplots()
            self.figs.append(fig)
            self.axs.append(ax)

        
        for i in range(0,len(self.times)):
            self.line1s.append(self.ax.plot(self.times[i], self.velocities[i], "b-"))
            self.line2s.append(self.ax.plot(self.times[i], self.targetVelocities[i], "b-"))

        (self.line1,) = self.ax.plot(self.x_data, self.y_data, "b-")
        (self.line2,) = self.ax.plot(self.x_data2, self.y_data2, "b-")

        self.fig.show()

        self.subscription

        self.beginning = self.get_clock().now()

    def plotGraph(self, x, y, setpoint,motor_id):
        print("Graphing")
        self.times[motor_id].append(x)
        self.velocities[motor_id].append(y)
        self.targetVelocities[motor_id].append(setpoint)

        self.line1s[motor_id].set_xdata(self.times[motor_id])
        self.line2s[motor_id].set_xdata(self.times[motor_id])

        self.line1s[motor_id].set_ydata(self.velocities[motor_id])
        self.line2s[motor_id].set_ydata(self.targetVelocities[motor_id])

        self.axs[motor_id].relim()
        self.axs[motor_id].autoscale_view()
        self.figs[motor_id].canvas.draw()
        self.figs[motor_id].canvas.flush_events()

        plt.pause(0.001)

    def plotter_callback(self, msg):
        self.get_logger().info(f"Velocity recieved: {msg.velocities[0]}")
        now = self.get_clock().now()
        
        if (self.initiated == False):
            self.names = msg.names
            for i in msg.names:
                self.axs[i].set_title(i)

        print("There is something running in the callback")
        delta = now - self.beginning
        x = delta.nanoseconds / 1e9
        for i in range(0,len(msg.names)):
            self.plotGraph(x, float(msg.velocities[i]), self.targetVelocities[i],i)

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
