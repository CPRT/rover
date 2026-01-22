#include "drive.hpp"

drive::drive() : Node("drive_node") {
  twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "controller_b/joy", 10,
      std::bind(&drive::drive_control, this, std::placeholders::_1));
};

void drive::drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = joystickMsg->axes[kForwardAxis] * kMaxLinear;
  twist.angular.z = joystickMsg->axes[kYawAxis] * kMaxAngular;
  twist_pub->publish(twist);
};
