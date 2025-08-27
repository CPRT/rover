#include "ArmIKMode.hpp"

#include "ArmHelpers.hpp"
#include "std_msgs/msg/bool.hpp"

ArmIKMode::ArmIKMode(rclcpp::Node* node) : Mode("IK Arm", node) {
  RCLCPP_INFO(node_->get_logger(), "IK Arm Mode");
  loadParameters();

  servo_client_ =
      node_->create_client<interfaces::srv::MoveServo>("servo_service");
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
  if (!ArmHelpers::start_moveit_servo(node_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to start MoveIt servo service, IK mode will not work");
  }

  auto stop_hw_interface_pub =
      node_->create_publisher<std_msgs::msg::Bool>("/arm_active", 10);
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  stop_hw_interface_pub->publish(msg);

  frame_to_publish_ = CAM_FRAME_ID;
  kServoMin = 0;
  kServoMax = 180;
  kClawMax = 63;
  kClawMin = 8;
  servoPos_ = kClawMax;
  buttonPressed_ = false;
  swapButton_ = false;
}

void ArmIKMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
  handleGripper(joystickMsg);
}

void ArmIKMode::handleTwist(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = node_->now();
  twist_msg.header.frame_id = frame_to_publish_;

  twist_msg.twist.linear.x = joystickMsg->axes[kxAxis];
  twist_msg.twist.linear.y = joystickMsg->axes[kyAxis];
  twist_msg.twist.linear.z =
      joystickMsg->buttons[kUpBut] - joystickMsg->buttons[kDownBut];
  twist_msg.twist.angular.x = joystickMsg->axes[kAroundX];
  twist_msg.twist.angular.y = joystickMsg->axes[kAroundY];
  twist_msg.twist.angular.z = joystickMsg->axes[kAroundZ];

  if (joystickMsg->buttons[kBase] == 1 && !swapButton_) {
    frame_to_publish_ = BASE_FRAME_ID;
    swapButton_ = true;
  } else if (joystickMsg->buttons[kEEF] == 1 && !swapButton_) {
    frame_to_publish_ = CAM_FRAME_ID;
    swapButton_ = true;
  } else if (!joystickMsg->buttons[kEEF] == 1 &&
             !joystickMsg->buttons[kBase] == 1) {
    swapButton_ = false;
  }
  twist_pub_->publish(twist_msg);
}
void ArmIKMode::handleGripper(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
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
}

void ArmIKMode::declareParameters(rclcpp::Node* node) {
  node->declare_parameter("arm_ik_mode.x_axis", 0);
  node->declare_parameter("arm_ik_mode.y_axis", 1);
  node->declare_parameter("arm_ik_mode.up_button", 2);
  node->declare_parameter("arm_ik_mode.down_button", 3);
  node->declare_parameter("arm_ik_mode.rotate_around_y", 4);
  node->declare_parameter("arm_ik_mode.rotate_around_x", 5);
  node->declare_parameter("arm_ik_mode.rotate_around_z", 6);
  node->declare_parameter("arm_ik_mode.open_claw", 7);
  node->declare_parameter("arm_ik_mode.close_claw", 8);
  node->declare_parameter("arm_ik_mode.base_frame", 9);
  node->declare_parameter("arm_ik_mode.eef_frame", 10);
  node->declare_parameter("arm_ik_mode.servo_port", 2);
}

void ArmIKMode::loadParameters() {
  node_->get_parameter("arm_ik_mode.x_axis", kxAxis);
  node_->get_parameter("arm_ik_mode.y_axis", kyAxis);
  node_->get_parameter("arm_ik_mode.up_button", kUpBut);
  node_->get_parameter("arm_ik_mode.down_button", kDownBut);
  node_->get_parameter("arm_ik_mode.rotate_around_y", kAroundY);
  node_->get_parameter("arm_ik_mode.rotate_around_x", kAroundX);
  node_->get_parameter("arm_ik_mode.rotate_around_z", kAroundZ);
  node_->get_parameter("arm_ik_mode.open_claw", kClawOpen);
  node_->get_parameter("arm_ik_mode.close_claw", kClawClose);
  node_->get_parameter("arm_ik_mode.base_frame", kBase);
  node_->get_parameter("arm_ik_mode.eef_frame", kEEF);
  node_->get_parameter("arm_ik_mode.servo_port", kServoPort);
}

interfaces::srv::MoveServo::Response ArmIKMode::sendRequest(int port,
                                                            int pos) const {
  auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
  request->port = port;
  request->pos = pos;

  // Wait for the service to be available
  if (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available after waiting");
    return interfaces::srv::MoveServo::Response();
  }

  auto future = servo_client_->async_send_request(request);

  // Wait for the result (with timeout)
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed");
    return interfaces::srv::MoveServo::Response();
  }

  return *future.get();
}

void ArmIKMode::servoRequest(int req_port, int req_pos) const {
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