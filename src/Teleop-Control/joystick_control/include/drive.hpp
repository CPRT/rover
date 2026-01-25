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

  void declare_parameters();
  void load_parameters();

  // Parameters
  double kMaxLinear;
  double kMaxAngular;
  int kForwardAxis;
  int kYawAxis;
  int kStrafeAxis;

  bool initialized_;
};

#endif // DRIVE_HPP