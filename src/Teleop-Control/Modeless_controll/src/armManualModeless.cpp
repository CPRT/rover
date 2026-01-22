#include "armManualModeless.hpp"
#include "ArmHelpers.hpp"
#include "std_msgs/msg/bool.hpp"
#include <algorithm>

arm_manual::arm_manual() : rclcpp::Node("modeless_arm_manual_control") {

  this->declareParameters();
  this->loadParameters();

  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/controller_b/joy", 10,
      std::bind(&arm_manual::processJoystickInput, this,
                std::placeholders::_1));

  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  servo_client_ =
      this->create_client<interfaces::srv::MoveServo>("servo_service");
  if (!ArmHelpers::start_moveit_servo(this)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start MoveIt servo service, Manual mode will not work");
  }

  auto stop_hw_interface_pub =
      this->create_publisher<std_msgs::msg::Bool>("/arm_active", 10);
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  stop_hw_interface_pub->publish(msg);

  kServoMin = 0;
  kServoMax = 180;

  kServoMin = 0;
  kServoMax = 180;
  kClawMax = 63;
  kClawMin = 8;
  servoPos_ = kClawMax;
  buttonPressed_ = false;
  joint_msg_ = control_msgs::msg::JointJog();
  joint_msg_.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                            "Joint_4", "Joint_5", "Joint_6"};
  joint_msg_.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void arm_manual::processJoystickInput(
    const sensor_msgs::msg::Joy::SharedPtr joystickMsg) {
  handleArm(joystickMsg);
}

void arm_manual::servoRequest(int req_port, int req_pos) {
  // If this is just a wrapper for setServoPosition:
  setServoPosition(req_port, req_pos);
}

double arm_manual::getThrottleValue(
    const sensor_msgs::msg::Joy::SharedPtr joystickMsg) {
  if (kThrottleAxis != -1) {
    double throttle = joystickMsg->axes[kThrottleAxis];
    throttle = std::max(kThrottleMin, std::min(kThrottleMax, throttle));
    // Normalize the throttle value to be between 0 and 1
    return (throttle - kThrottleMin) / (kThrottleMax - kThrottleMin);
  }
  return 1.0;
}

void arm_manual::setServoPosition(int port, int position) {
  if (camera_service_available_) {
    auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
    request->port = port;
    request->pos = position;

    servo_client_->async_send_request(request);
  }
}

void arm_manual::handleArm(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  double throttle = getThrottleValue(joystickMsg);
  joint_msg_.header = joystickMsg->header;

  // Base (might want a deadzone. TBD)
  joint_msg_.velocities[0] = -joystickMsg->axes[kBaseAxis] * throttle;

  // Simple straight movement (NOT inverse kin)
  // some scaling to move in a straight line
  if (joystickMsg->buttons[kSimpleForward] == 1) {
    joint_msg_.velocities[1] = -0.78 * throttle;
    joint_msg_.velocities[2] = 0.92 * throttle;
    joint_msg_.velocities[4] = -1.0 * throttle;
  } else if (joystickMsg->buttons[kSimpleBackward] == 1) {
    joint_msg_.velocities[1] = 0.90 * throttle;
    joint_msg_.velocities[2] = -0.80 * throttle;
    joint_msg_.velocities[4] = 1.0 * throttle;
  } else {
    // act1
    if (joystickMsg->axes[kAct1Axis] > 0.1 ||
        joystickMsg->axes[kAct1Axis] < -0.1) {
      joint_msg_.velocities[1] = -joystickMsg->axes[kAct1Axis] * throttle;
    } else {
      joint_msg_.velocities[1] = 0;
    }

    // act2
    joint_msg_.velocities[2] = joystickMsg->axes[kAct2Axis] * throttle;
  }

  // Elbow
  // Deadzone, easy to turn this one by accident.
  if (joystickMsg->axes[kElbowYaw] < 0.2 &&
      joystickMsg->axes[kElbowYaw] > -0.2) {
    joint_msg_.velocities[3] = 0;
  } else {
    joint_msg_.velocities[3] = joystickMsg->axes[kElbowYaw] * throttle;
  }

  // Wrist Tilt
  if (joystickMsg->buttons[kSimpleForward] == 0 &&
      joystickMsg->buttons[kSimpleForward] == 0) {
    joint_msg_.velocities[4] = (joystickMsg->buttons[kWristYawPositive] -
                                joystickMsg->buttons[kWristYawNegative]) *
                               throttle;
  }

  // Wrist Turn
  joint_msg_.velocities[5] = joystickMsg->axes[kWristRoll] * throttle;

  // Gripper. Will cycle between open, half open, and close on button release.
  if (joystickMsg->buttons[kClawOpen] == 1 && !buttonPressed_) {
    if (servoPos_ + ((kClawMax - kClawMin) / 2) < kClawMax + 1) {
      buttonPressed_ = true;
      servoPos_ = servoPos_ + ((kClawMax - kClawMin) / 2);
      servoRequest(kServoPort, servoPos_);
    } else {
      buttonPressed_ = true;
      RCLCPP_INFO(this->get_logger(), "Max Open");
      RCLCPP_INFO(this->get_logger(), "%d", servoPos_);
    }
  } else if (joystickMsg->buttons[kClawClose] == 1 && !buttonPressed_) {
    if (servoPos_ - ((kClawMax - kClawMin) / 2) > kClawMin - 1) {
      buttonPressed_ = true;
      servoPos_ = servoPos_ - ((kClawMax - kClawMin) / 2);
      servoRequest(kServoPort, servoPos_);
    } else {
      buttonPressed_ = true;
      RCLCPP_INFO(this->get_logger(), "Max Close");
      RCLCPP_INFO(this->get_logger(), "%d", servoPos_);
    }
  } else if ((joystickMsg->buttons[kClawClose] == 0) &&
             (joystickMsg->buttons[kClawOpen] == 0)) {
    buttonPressed_ = false;
  }

  joint_pub_->publish(joint_msg_);
}

void arm_manual::declareParameters() {
  this->declare_parameter("arm_manual_mode.base_axis", 0);
  this->declare_parameter("arm_manual_mode.wrist_roll", 1);
  this->declare_parameter("arm_manual_mode.wrist_yaw_positive", 2);
  this->declare_parameter("arm_manual_mode.wrist_yaw_negative", 3);
  this->declare_parameter("arm_manual_mode.act1_axis", 4);
  this->declare_parameter("arm_manual_mode.act2_axis", 5);
  this->declare_parameter("arm_manual_mode.elbow_yaw", 6);
  this->declare_parameter("arm_manual_mode.claw_open", 8);
  this->declare_parameter("arm_manual_mode.claw_close", 9);
  this->declare_parameter("arm_manual_mode.simple_forward", 10);
  this->declare_parameter("arm_manual_mode.simple_backward", 11);
  this->declare_parameter("arm_manual_mode.servo_port", 12);
  this->declare_parameter("arm_manual_mode.throttle.axis", 7);
  this->declare_parameter("arm_manual_mode.throttle.min", -1.0);
  this->declare_parameter("arm_manual_mode.throttle.max", 1.0);
}

void arm_manual::loadParameters() {
  this->get_parameter("arm_manual_mode.base_axis", kBaseAxis);
  this->get_parameter("arm_manual_mode.wrist_roll", kWristRoll);
  this->get_parameter("arm_manual_mode.wrist_yaw_positive", kWristYawPositive);
  this->get_parameter("arm_manual_mode.wrist_yaw_negative", kWristYawNegative);
  this->get_parameter("arm_manual_mode.act1_axis", kAct1Axis);
  this->get_parameter("arm_manual_mode.act2_axis", kAct2Axis);
  this->get_parameter("arm_manual_mode.elbow_yaw", kElbowYaw);
  this->get_parameter("arm_manual_mode.claw_open", kClawOpen);
  this->get_parameter("arm_manual_mode.claw_close", kClawClose);
  this->get_parameter("arm_manual_mode.simple_forward", kSimpleForward);
  this->get_parameter("arm_manual_mode.simple_backward", kSimpleBackward);
  this->get_parameter("arm_manual_mode.servo_port", kServoPort);
  this->get_parameter("arm_manual_mode.throttle.axis", kThrottleAxis);
  this->get_parameter("arm_manual_mode.throttle.max", kThrottleMax);
  this->get_parameter("arm_manual_mode.throttle.min", kThrottleMin);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Create the node (NoMode now inherits from rclcpp::Node)
  auto node = std::make_shared<arm_manual>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
