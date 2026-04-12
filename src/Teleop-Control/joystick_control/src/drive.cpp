#include "drive.hpp"

drive::drive() : Node("drive_node"), initialized_(false) {
  declare_parameters();
  load_parameters();
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  servo_y_pub_ = this->create_publisher<std_msgs::msg::Float32>("/tilt", 10);
  servo_x_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pan", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&drive::drive_control, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Drive controller started");
  servo_y_ = kDefaultServoY;
  servo_x_ = kDefaultServoX;
};

void drive::drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (!initialized_) {
    if (std::abs(joystickMsg->axes[kForwardAxis]) < kJoyDeadzone &&
        std::abs(joystickMsg->axes[kStrafeAxis]) < kJoyDeadzone &&
        std::abs(joystickMsg->axes[kYawAxis]) < kJoyDeadzone) {
      initialized_ = true;
    }
    return;
  }
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = joystickMsg->axes[kForwardAxis] * kMaxLinear;
  twist.linear.y = joystickMsg->axes[kStrafeAxis] * kMaxLinear;
  twist.angular.z = joystickMsg->axes[kYawAxis] * kMaxAngular;

  twist_pub_->publish(twist);

  if (joystickMsg->buttons[kServoHomeButton]) {
    servo_y_ = kDefaultServoY;
    servo_x_ = kDefaultServoX;
  }

  if (joystickMsg->axes[kServoYAxis] > kJoyDeadzone) {
    servo_y_ -= kServoIncrement;
  } else if (joystickMsg->axes[kServoYAxis] < -kJoyDeadzone) {
    servo_y_ += kServoIncrement;
  }
  if (joystickMsg->axes[kServoXAxis] > kJoyDeadzone) {
    servo_x_ += kServoIncrement;
  } else if (joystickMsg->axes[kServoXAxis] < -kJoyDeadzone) {
    servo_x_ -= kServoIncrement;
  }
  auto servo_msg = std_msgs::msg::Float32();
  servo_y_ = std::clamp(servo_y_, kServoMin, kServoMax);
  servo_msg.data = servo_y_;
  servo_y_pub_->publish(servo_msg);

  servo_x_ = std::clamp(servo_x_, kServoMin, kServoMax);
  servo_msg.data = servo_x_;
  servo_x_pub_->publish(servo_msg);
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
  this->declare_parameter("servo_increment", 0.01);
  this->declare_parameter("servo_min", -3.14);
  this->declare_parameter("servo_max", 3.14);
  this->declare_parameter("joy_deadzone", 0.01);
  this->declare_parameter("default_servo_x", 0.0);
  this->declare_parameter("default_servo_y", 0.0);
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
  this->get_parameter("servo_increment", kServoIncrement);
  this->get_parameter("servo_min", kServoMin);
  this->get_parameter("servo_max", kServoMax);
  this->get_parameter("joy_deadzone", kJoyDeadzone);
  this->get_parameter("default_servo_x", kDefaultServoX);
  this->get_parameter("default_servo_y", kDefaultServoY);

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