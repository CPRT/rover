import math
import threading
from ros_phoenix.msg import MotorStatus
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time as time
import matplotlib
import time as pytime

matplotlib.use("TkAgg")  # make sure we have an interactive backend

import matplotlib.pyplot as plt
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from ament_index_python.packages import get_package_share_directory
import os

plt.rcParams["toolbar"] = "none"


"""
For running this, there are some things you may want to note:
    - You have to start the Joy node with ros2 run joy joy_node
    - This is specifically for the base motor
    - If you close the window, the program stops because python lkw just like that
"""
# NOTE PLEASE REFACTOR WITH WHAT CONNOR SAID (YOU ARE GOING TO GET LISTS AND STUFF)


class JointData:
    def __init__(self, name, ax):
        self.name = name
        self.number = int(name.split("_")[1]) - 1
        self.time = 0
        self.targetVelocity = 0
        self.velocity = 0
        self.times = []
        self.velocities = []
        self.targetVelocitiesHistory = []
        self.targetVelocity = 0
        self.ax = ax
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Velocity (rad/s)")
        self.ax.set_title(name)
        self.ax.legend(["Velocity", "Target Velocity"])
        self.ax.grid()
        self.line1 = self.ax.plot([], [], "-", label="Velocity")[0]
        self.line2 = self.ax.plot([], [], "--", label="Target")[0]
        
        seconds = 30  # Number of seconds to display on the graph
        
        self.MAX_POINTS = seconds*10  # Maximum number of points to display on the graph

        self.ax.legend()

        #self.fig.show()

    def plotGraph(self):
        self.targetVelocitiesHistory.append(self.targetVelocity)
        self.times.append(self.time)
        self.velocities.append(self.velocity)
        
        if len(self.times) > self.MAX_POINTS:
            self.times.pop(0)
            self.velocities.pop(0)
            self.targetVelocitiesHistory.pop(0)

        self.line1.set_xdata(self.times)
        self.line2.set_xdata(self.times)

        self.line1.set_ydata(self.velocities)
        self.line2.set_ydata(self.targetVelocitiesHistory)

        self.ax.relim()
        self.ax.autoscale_view()

class BasePIDGrapher(Node):
    
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
            JointState,
            "/joint_states",
            self.plotter_callback,
            10,
        )
        self.subscription_targets = self.create_subscription(
            JointTrajectory,
            "/rover_arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )

        self.targetVelocity = 10

        plt.ion()
        
        joint_names = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]

        
        n_plots = len(joint_names)
        cols = 2
        rows = math.ceil(n_plots / cols)

        self.fig, self.axs = plt.subplots(rows, cols, figsize=(12, rows*3))
        
        #self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 8))  # 3 rows × 2 columns
        self.fig.subplots_adjust(left=0.07, right=0.95, top=0.95, bottom=0.07, hspace=0.3, wspace=0.3)
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
                Joint.velocity = msg.velocity[i]


    def _show_plot(self):
        plt.show(block=True)

    def trajectory_callback(self, msg):
        if len(msg.points) > 0:
            point = msg.points[-1]  # Get the last point in the trajectory
            for i in range(0, len(msg.joint_names)):
                jointName = msg.joint_names[i]
                self.Joints[jointName].targetVelocity = point.velocities[i]
    
    def on_key(self, event):
        if event.key == "s":
            self.save()
            print("Saved plot")
    
    def save(self):
        # OR force the src directory like this:
        script_dir = os.path.join(os.getcwd(), 'pid_grapher')
        plots_dir = os.path.join(script_dir, "plots")
        os.makedirs(plots_dir, exist_ok=True)

        save_path = os.path.join(plots_dir, "joint_plot.png")
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"Plot saved to {save_path}")
    """
    TODO this mapping is very fucked but I don't have it in me right now so ill figure this logic out later
    perhaps a dictionary? Oh wait, that's actually not a bad idea but I would have to init everything after the names come in?
    Fuck it, that's a good idea, sounds like a future me problem, I have midterms to study for
    """


def main(args=None):
    rclpy.init(args=args)
    node = BasePIDGrapher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
