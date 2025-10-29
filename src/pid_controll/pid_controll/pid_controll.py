import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from ros_phoenix.msg import MotorControl
from std_msgs.msg import String
import json


class motorControll:
    current_state = 0.0
    requested_state = 0.0
    init_state = 0.0
    initialized = False

    def request_state_update(self, delta):
        self.requested_state = self.requested_state + delta

    def initialize(self):
        self.requested_state = self.init_state
        self.initialized = True


class pidControll(Node):
    mode = "position"

    initiated = False

    names = ["base", "act1", "elbow", "wristTilt", "wristTurn", "act2"]

    motors = {}

    state_publishers = {}

    button_states_prev = []

    def __init__(self):
        super().__init__("position_tuner")
        self.get_logger().info("Tuner Node has been started.")

        self.declare_parameter("mode", 1)
        modeInt = self.get_parameter("mode").get_parameter_value().integer_value

        if modeInt == 1:
            self.mode = "position"
        elif modeInt == 2:
            self.mode = "velocity"
        else:
            self.get_logger().error("Invalid mode parameter, defaulting to position")
            self.mode = "position"

        self.get_logger().info(f"Operating in {self.mode} mode.")

        for name in self.names:
            publisher = self.create_publisher(MotorControl, "/" + name + "/set", 10)
            self.state_publishers[name] = publisher

        self.state_target_publisher = self.create_publisher(
            String, "/targets/" + self.mode, 10
        )

        self.timer = self.create_timer(0.01, self.publish_states)

        self.subscription_joy = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )
        self.subscription_states = self.create_subscription(
            String, "/mStates/" + self.mode, self.state_callback, 10
        )
        self.get_logger().info(f"Subscribed to /mStates/{self.mode}")

        for i in self.names:
            self.motors[i] = motorControll()

    def state_callback(self, msg: String):
        data = json.loads(msg.data)
        for key in data:
            if key in self.motors:
                self.motors[key].current_state = data[key]
        # self.get_logger().info(f"Updated states: {self.current_states}")

    def publish_states(self):
        requested_states = {}
        for key in self.motors:
            motor_command = MotorControl()
            if self.mode == "position":
                motor_command.mode = MotorControl.POSITION
            elif self.mode == "velocity":
                motor_command.mode = MotorControl.VELOCITY
            else:
                raise ValueError("Invalid mode selected")
            motor_command.value = self.motors[key].requested_state
            if self.motors[key].initialized:
                self.state_publishers[key].publish(motor_command)
            requested_states[key] = self.motors[key].requested_state
        self.state_target_publisher.publish(String(data=json.dumps(requested_states)))

    """
    Actually wait this is also stupid because of arm jittering. I have actually no idea how to even do this properly/ Darren said something about 
    sending it super far until the joystick says neutral but then the pid graphs are gonna be fucked. Oh well :sob: I suppose I will just lkw set positions and see what happens
    perhaps I can use the buttons? idk
    Actually wait the buttons are a fire idea this could be peak
    """

    def joy_callback(self, msg: Joy):

        for key in self.motors:
            if not self.motors[key].initialized:
                self.initiated = False
                break
            self.initiated = True

        if self.initiated:
            for i in range(6, len(self.names) + 6):
                if msg.buttons[i] == 0 and self.button_states_prev[i] == 1:
                    motor_name = self.names[i - 6]
                    delta = msg.axes[2] * 0.1  # Scale the joystick input
                    self.motors[motor_name].request_state_update(delta)
                    self.get_logger().info(
                        f"Updated {motor_name} requested state to {self.motors[motor_name].requested_state}"
                    )

        else:
            if msg.buttons[6] == 0 and self.button_states_prev[6] == 1:
                for key in self.motors:
                    self.motors[key].init_state = self.motors[key].current_state
                    self.motors[key].initialize()
                    self.get_logger().info(
                        f"Initialized {key} to {self.motors[key].current_state}"
                    )

        self.button_states_prev = msg.buttons.copy()

        # I need a fuction to run an on release type deal here


def main(args=None):
    rclpy.init(args=args)
    node = pidControll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
