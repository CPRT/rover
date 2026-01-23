#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class drive : public rclcpp::Node {
public:
  drive();
  void drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  int kForwardAxis = 1;     // Example axis index for forward/backward
  int kYawAxis = 0;         // Example axis index for left/right
  double kMaxLinear = 2.0;  // Max linear speed
  double kMaxAngular = 2.0; // Max angular speed
};

#endif // DRIVE_HPP