#include "drive.hpp"

drive::drive() : Node("drive_node") {
  declare_parameters();
  load_parameters();
  twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/controller_a/joy", 10,
      std::bind(&drive::drive_control, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Drive controller started");
};

void drive::drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = joystickMsg->axes[kForwardAxis] * kMaxLinear;
  twist.linear.y = joystickMsg->axes[kStrafeAxis] * kMaxLinear;
  twist.angular.z = joystickMsg->axes[kYawAxis] * kMaxAngular;

  twist_pub->publish(twist);
};

void drive::declare_parameters() {
  this->declare_parameter("max_linear", 1.0);
  this->declare_parameter("max_angular", 1.0);
  this->declare_parameter("forward_axis", 1);
  this->declare_parameter("yaw_axis", 2);
  this->declare_parameter("strafe_axis", 3);
}
void drive::load_parameters() {
  this->get_parameter("max_linear", kMaxLinear);
  this->get_parameter("max_angular", kMaxAngular);
  this->get_parameter("forward_axis", kForwardAxis);
  this->get_parameter("yaw_axis", kYawAxis);
  this->get_parameter("strafe_axis", kStrafeAxis);

  RCLCPP_INFO(this->get_logger(), "Loaded Max Linear: %f", kMaxLinear);
  RCLCPP_INFO(this->get_logger(), "Loaded Max Angular: %f", kMaxAngular);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Replace 'arm' with your actual class name if different
  auto node = std::make_shared<drive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}