#include "ArmManualMode.hpp"

#include "ArmHelpers.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "std_msgs/msg/bool.hpp"

ArmManualMode::ArmManualMode(rclcpp::Node* node) : Mode("Manual Arm", node) {
  RCLCPP_INFO(node_->get_logger(), "Arm Manual Mode");
  loadParameters();
  joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  servo_client_ =
      node_->create_client<interfaces::srv::MoveServo>("servo_service");
  if (!ArmHelpers::start_moveit_servo(node_)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to start MoveIt servo service, Manual mode will not work");
  }

  auto stop_hw_interface_pub =
      node_->create_publisher<std_msgs::msg::Bool>("/arm_active", 10);
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  stop_hw_interface_pub->publish(msg);

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

double ArmManualMode::getThrottleValue(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  if (kThrottleAxis != -1) {
    double throttle = joystickMsg->axes[kThrottleAxis];
    throttle = std::max(kThrottleMin, std::min(kThrottleMax, throttle));
    // Normalize the throttle value to be between 0.2 and 1.0
    return 0.2 +
           ((throttle - kThrottleMin) / (kThrottleMax - kThrottleMin) * 0.80);
  }
  return 1.0;
}

void ArmManualMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
}

void ArmManualMode::handleTwist(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
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
      RCLCPP_INFO(node_->get_logger(), "Max Open");
      RCLCPP_INFO(node_->get_logger(), "%d", servoPos_);
    }
  } else if (joystickMsg->buttons[kClawClose] == 1 && !buttonPressed_) {
    if (servoPos_ - ((kClawMax - kClawMin) / 2) > kClawMin - 1) {
      buttonPressed_ = true;
      servoPos_ = servoPos_ - ((kClawMax - kClawMin) / 2);
      servoRequest(kServoPort, servoPos_);
    } else {
      buttonPressed_ = true;
      RCLCPP_INFO(node_->get_logger(), "Max Close");
      RCLCPP_INFO(node_->get_logger(), "%d", servoPos_);
    }
  } else if ((joystickMsg->buttons[kClawClose] == 0) &&
             (joystickMsg->buttons[kClawOpen] == 0)) {
    buttonPressed_ = false;
  }

  joint_pub_->publish(joint_msg_);
}

void ArmManualMode::declareParameters(rclcpp::Node* node) {
  node->declare_parameter("arm_manual_mode.base_axis", 0);
  node->declare_parameter("arm_manual_mode.wrist_roll", 1);
  node->declare_parameter("arm_manual_mode.wrist_yaw_positive", 2);
  node->declare_parameter("arm_manual_mode.wrist_yaw_negative", 3);
  node->declare_parameter("arm_manual_mode.act1_axis", 4);
  node->declare_parameter("arm_manual_mode.act2_axis", 5);
  node->declare_parameter("arm_manual_mode.elbow_yaw", 6);
  node->declare_parameter("arm_manual_mode.claw_open", 8);
  node->declare_parameter("arm_manual_mode.claw_close", 9);
  node->declare_parameter("arm_manual_mode.simple_forward", 10);
  node->declare_parameter("arm_manual_mode.simple_backward", 11);
  node->declare_parameter("arm_manual_mode.servo_port", 12);
  node->declare_parameter("arm_manual_mode.throttle.axis", 7);
  node->declare_parameter("arm_manual_mode.throttle.min", -1.0);
  node->declare_parameter("arm_manual_mode.throttle.max", 1.0);
}

void ArmManualMode::loadParameters() {
  node_->get_parameter("arm_manual_mode.base_axis", kBaseAxis);
  node_->get_parameter("arm_manual_mode.wrist_roll", kWristRoll);
  node_->get_parameter("arm_manual_mode.wrist_yaw_positive", kWristYawPositive);
  node_->get_parameter("arm_manual_mode.wrist_yaw_negative", kWristYawNegative);
  node_->get_parameter("arm_manual_mode.act1_axis", kAct1Axis);
  node_->get_parameter("arm_manual_mode.act2_axis", kAct2Axis);
  node_->get_parameter("arm_manual_mode.elbow_yaw", kElbowYaw);
  node_->get_parameter("arm_manual_mode.claw_open", kClawOpen);
  node_->get_parameter("arm_manual_mode.claw_close", kClawClose);
  node_->get_parameter("arm_manual_mode.simple_forward", kSimpleForward);
  node_->get_parameter("arm_manual_mode.simple_backward", kSimpleBackward);
  node_->get_parameter("arm_manual_mode.servo_port", kServoPort);
  node_->get_parameter("arm_manual_mode.throttle.axis", kThrottleAxis);
  node_->get_parameter("arm_manual_mode.throttle.max", kThrottleMax);
  node_->get_parameter("arm_manual_mode.throttle.min", kThrottleMin);
}

void ArmManualMode::servoRequest(int req_port, int req_pos) const {
  auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
  request->port = req_port;
  request->pos = req_pos;

  if (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available");
    return;
  }

  // Simple callback that just logs errors
  auto callback =
      [this](rclcpp::Client<interfaces::srv::MoveServo>::SharedFuture future) {
        try {
          auto response = future.get();
          if (!response->status) {
            RCLCPP_ERROR(node_->get_logger(), "Servo move failed");
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s",
                       e.what());
        }
      };

  servo_client_->async_send_request(request, callback);
}