#include "drive.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

drive::drive() : Node("drive_node"), initialized_(false) {
  declare_parameters();
  load_parameters();
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  gimbal_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/gimbal_controller/cmd_vel", 1);
  gimbal_reset_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/gimbal_controller/reset", 1);

  camera_client_ =
      this->create_client<interfaces::srv::VideoOut>("/start_video");

  auto presets_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  presets_sub_ = this->create_subscription<interfaces::msg::VideoPresets>(
      "/video_presets", presets_qos,
      std::bind(&drive::presets_cb, this, std::placeholders::_1));

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(2).best_effort(),
      std::bind(&drive::drive_control, this, std::placeholders::_1));
  drive_throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/drive_throttle", 10,
      std::bind(&drive::drive_throttle_cb, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Drive controller started");
  drive_throttle_ = 1.0f;
};

void drive::presets_cb(
    const interfaces::msg::VideoPresets::SharedPtr presets_msg) {
  video_carousell_ = presets_msg->presets;
  video_carousell_idx_ = 0;

  RCLCPP_INFO(this->get_logger(), "Received %zu video presets",
              video_carousell_.size());
}

void drive::drive_throttle_cb(
    const std_msgs::msg::Float32::SharedPtr throttle_msg) {
  drive_throttle_ = std::clamp(throttle_msg->data, 0.0f, 1.0f);

  RCLCPP_INFO(this->get_logger(), "Drive throttle set to %.2f",
              drive_throttle_);
}

void drive::camera_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  const auto &left_but = joystickMsg->buttons[kCamLeftBut];
  const auto &right_but = joystickMsg->buttons[kCamRightBut];

  if (!left_but && !right_but) {
    cam_debounce_ = false;
    return;
  }
  // If button has not been released
  if (cam_debounce_) {
    return;
  }
  cam_debounce_ = true;

  if (video_carousell_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No video presets have been received");
    return;
  }

  if (left_but) {
    if (video_carousell_idx_ == 0) {
      video_carousell_idx_ = video_carousell_.size();
    }
    --video_carousell_idx_;
  }
  if (right_but) {
    if (++video_carousell_idx_ >= video_carousell_.size()) {
      video_carousell_idx_ = 0;
    }
  }
  auto request = std::make_shared<interfaces::srv::VideoOut::Request>();
  request->sources = video_carousell_[video_carousell_idx_].sources;
  RCLCPP_INFO(this->get_logger(), "Sending Video preset %s",
              video_carousell_[video_carousell_idx_].name.c_str());
  camera_client_->async_send_request(request);
}

void drive::drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  // Controller mappings differ; malformed/disconnected Joy input must not index
  // past the arrays.
  const auto valid_index = [](int index, size_t size) {
    return index >= 0 && static_cast<size_t>(index) < size;
  };
  bool valid = true;
  for (int axis :
       {kForwardAxis, kStrafeAxis, kYawAxis, kServoXAxis, kServoYAxis}) {
    valid = valid && valid_index(axis, joystickMsg->axes.size());
    if (valid)
      valid = std::isfinite(joystickMsg->axes[axis]);
  }
  for (int button : {kLockTurnBut, kCamLeftBut, kCamRightBut, kServoHomeButton,
                     kMastLeftButton, kMastRightButton})
    valid = valid && valid_index(button, joystickMsg->buttons.size());
  if (!valid) {
    twist_pub_->publish(geometry_msgs::msg::Twist());
    gimbal_pub_->publish(geometry_msgs::msg::Twist());
    initialized_ = false;
    return;
  }
  if (!initialized_) {
    if (std::abs(joystickMsg->axes[kForwardAxis]) < kJoyDeadzone &&
        std::abs(joystickMsg->axes[kStrafeAxis]) < kJoyDeadzone &&
        std::abs(joystickMsg->axes[kYawAxis]) < kJoyDeadzone) {
      initialized_ = true;
    }
    return;
  }
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x =
      joystickMsg->axes[kForwardAxis] * kMaxLinear * drive_throttle_;
  twist.linear.y =
      joystickMsg->axes[kStrafeAxis] * kMaxLinear * drive_throttle_;
  twist.angular.z = joystickMsg->axes[kYawAxis] * kMaxAngular * drive_throttle_;

  if (joystickMsg->buttons[kLockTurnBut]) {
    twist.angular.z = 0.0;
    twist.linear.y = 0.0;
  }

  twist_pub_->publish(twist);
  camera_control(joystickMsg);
  auto reset = std_msgs::msg::Bool();
  reset.data = joystickMsg->buttons[kServoHomeButton] != 0;
  gimbal_reset_pub_->publish(reset);
  auto gimbal = geometry_msgs::msg::Twist();
  if (!reset.data) {
    const auto velocity = [this](double axis) {
      return std::abs(axis) > kJoyDeadzone
                 ? std::clamp(axis, -1.0, 1.0) * kGimbalSpeed
                 : 0.0;
    };
    gimbal.angular.z = velocity(joystickMsg->axes[kServoXAxis]);
    gimbal.angular.y = -velocity(joystickMsg->axes[kServoYAxis]);
    if (joystickMsg->buttons[kMastLeftButton])
      gimbal.angular.x = -kGimbalSpeed;
    else if (joystickMsg->buttons[kMastRightButton])
      gimbal.angular.x = kGimbalSpeed;
  }
  gimbal_pub_->publish(gimbal);
};

void drive::declare_parameters() {
  this->declare_parameter("max_linear", 1.0);
  this->declare_parameter("max_angular", 1.0);
  this->declare_parameter("forward_axis", 1);
  this->declare_parameter("yaw_axis", 2);
  this->declare_parameter("strafe_axis", 3);
  this->declare_parameter("servo_y_axis", 4);
  this->declare_parameter("servo_x_axis", 5);
  this->declare_parameter("servo_home_button", 8);
  this->declare_parameter("mast_left_button", 3);
  this->declare_parameter("mast_right_button", 1);
  this->declare_parameter("cam_left_button", 4);
  this->declare_parameter("cam_right_button", 5);
  this->declare_parameter("lock_turn_button", 6);
  this->declare_parameter("joy_deadzone", 0.01);
  this->declare_parameter("gimbal_speed", 0.4);
}

void drive::load_parameters() {
  this->get_parameter("max_linear", kMaxLinear);
  this->get_parameter("max_angular", kMaxAngular);
  this->get_parameter("forward_axis", kForwardAxis);
  this->get_parameter("yaw_axis", kYawAxis);
  this->get_parameter("strafe_axis", kStrafeAxis);
  this->get_parameter("servo_y_axis", kServoYAxis);
  this->get_parameter("servo_x_axis", kServoXAxis);
  this->get_parameter("servo_home_button", kServoHomeButton);
  this->get_parameter("mast_left_button", kMastLeftButton);
  this->get_parameter("mast_right_button", kMastRightButton);
  this->get_parameter("cam_left_button", kCamLeftBut);
  this->get_parameter("cam_right_button", kCamRightBut);
  this->get_parameter("lock_turn_button", kLockTurnBut);
  this->get_parameter("joy_deadzone", kJoyDeadzone);
  this->get_parameter("gimbal_speed", kGimbalSpeed);
  if (!std::isfinite(kGimbalSpeed) || kGimbalSpeed <= 0)
    throw std::invalid_argument("gimbal_speed must be finite and positive");

  RCLCPP_INFO(this->get_logger(), "Loaded Max Linear: %f", kMaxLinear);
  RCLCPP_INFO(this->get_logger(), "Loaded Max Angular: %f", kMaxAngular);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}