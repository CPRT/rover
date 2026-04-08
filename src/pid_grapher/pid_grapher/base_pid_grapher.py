import math
from datetime import datetime
from pid_grapher.joint_data import JointData
import rclpy
from rclpy.node import Node
import rclpy.time as time
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import os
from std_msgs.msg import Float64MultiArray

plt.rcParams["toolbar"] = "none"


class BasePIDGrapher(Node):
    def update_plots(self):
        rate = 10
        period = 1.0 / rate
        for j in self.Joints.values():
            j.plotGraph()
        self.fig.canvas.draw_idle()
        plt.pause(0.000000001)

    def __init__(self):
        super().__init__("base_pid_grapher")
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.plotter_callback,
            10,
        )
        self.subscription_targets = self.create_subscription(
            Float64MultiArray,
            "/arm_controller_servo/commands",
            self.trajectory_callback,
            10,
        )

        plt.ion()

        joint_names = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]

        n_plots = len(joint_names)
        cols = 2
        rows = math.ceil(n_plots / cols)

        self.fig, self.axs = plt.subplots(rows, cols, figsize=(12, rows * 3))

        # self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 8))  # 3 rows × 2 columns
        self.fig.subplots_adjust(
            left=0.07, right=0.95, top=0.95, bottom=0.07, hspace=0.3, wspace=0.3
        )
        self.axs = self.axs.flatten()  # make it easy to index as a list
        self.Joints = {}
        for name, ax in zip(joint_names, self.axs):
            self.Joints[name] = JointData(name, ax)

        plt.show(block=False)

        self.timer = self.create_timer(0.1, self.update_plots)

        self.subscription

        self.beginning = self.get_clock().now()

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def plotter_callback(self, msg):
        now = self.get_clock().now()

        delta = now - self.beginning
        x = delta.nanoseconds / 1e9
        for i in range(0, len(msg.name)):
            if msg.name[i] in self.Joints:
                Joint = self.Joints[msg.name[i]]
                Joint.time = x
                Joint.state = msg.position[i]

    def _show_plot(self):
        plt.show(block=True)

    def trajectory_callback(self, msg):
        if len(msg.data) > 0:
            jointNames = list(self.Joints.keys())
            for i in range(0, len(jointNames)):
                jointName = jointNames[i]
                self.Joints[jointName].target = msg.data[i]

    def on_key(self, event):
        if event.key == "p":
            script_dir = os.path.join(os.getcwd(), "pid_grapher")
            plots_dir = os.path.join(script_dir, "plots")
            os.makedirs(plots_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            save_path = os.path.join(plots_dir, f"joint_plot_{timestamp}.png")

            plt.savefig(save_path, bbox_inches="tight", dpi=300)
            print(f"Plot saved to {save_path}")
        elif event.key == "r":
            for j in self.Joints.values():
                j.reset()
            self.beginning = self.get_clock().now()
            print("Reset data")


def main(args=None):
    rclpy.init(args=args)
    node = BasePIDGrapher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
