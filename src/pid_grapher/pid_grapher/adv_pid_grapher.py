from datetime import datetime
import math
import rclpy
from rclpy.node import Node
import json
import matplotlib

matplotlib.use("TkAgg")  # make sure we have an interactive backend
import matplotlib.pyplot as plt
from std_msgs.msg import String
import os
from .joint_data import JointData


class advPIDGrapher(Node):
    mode = "position"
    Joints = {}

    def update_plots(self):
        rate = 10
        period = 1.0 / rate
        for j in self.Joints.values():
            j.plotGraph()
        self.fig.canvas.draw_idle()  # schedules one redraw for all 6 subplots
        plt.pause(0.000000001)

    def __init__(self):
        super().__init__("base_pid_grapher")
        self.subscription = self.create_subscription(
            String,
            "/mStates/" + self.mode,
            self.plotter_callback,
            10,
        )
        self.subscription_targets = self.create_subscription(
            String,
            "/targets/" + self.mode,
            self.trajectory_callback,
            10,
        )

        plt.ion()

        motor_names = ["base", "act1", "elbow", "wristTilt", "wristTurn", "act2"]

        n_plots = len(motor_names)
        cols = 2
        rows = math.ceil(n_plots / cols)

        self.fig, self.axs = plt.subplots(rows, cols, figsize=(12, rows * 3))

        self.fig.subplots_adjust(
            left=0.07, right=0.95, top=0.95, bottom=0.07, hspace=0.3, wspace=0.3
        )
        self.axs = self.axs.flatten()  # make it easy to index as a list
        for name, ax in zip(motor_names, self.axs):
            self.Joints[name] = JointData(name, ax)

        plt.show(block=False)

        self.timer = self.create_timer(0.1, self.update_plots)

        self.beginning = self.get_clock().now()

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def plotter_callback(self, msg: String):
        data = json.loads(msg.data)
        for key in data:
            if key in self.Joints:
                self.Joints[key].time = (
                    self.get_clock().now() - self.beginning
                ).nanoseconds / 1e9
                self.Joints[key].state = data[key]

    def trajectory_callback(self, msg: String):
        data = json.loads(msg.data)
        for key in data:
            if key in self.Joints:
                self.Joints[key].target = data[key]

    def on_key(self, event):
        if event.key == "p":
            script_dir = os.path.join(os.getcwd(), "pid_grapher")
            plots_dir = os.path.join(script_dir, "plots")
            os.makedirs(plots_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            save_path = os.path.join(plots_dir, f"joint_plot_{timestamp}.png")

            plt.savefig(save_path, bbox_inches="tight", dpi=300)
            print(f"Plot saved to {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = advPIDGrapher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
