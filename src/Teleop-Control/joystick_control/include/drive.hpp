#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class drive : public rclcpp::Node {
public:
  drive();
  void drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_x_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void declare_parameters();
  void load_parameters();

  // Parameters
  double kMaxLinear;
  double kMaxAngular;
  int kForwardAxis;
  int kYawAxis;
  int kStrafeAxis;
  int kServoYAxis;
  int kServoXAxis;
  int kServoHomeButton;
  double kServoIncrement;
  double kServoMin;
  double kServoMax;
  double kJoyDeadzone;
  double kDefaultServoX;
  double kDefaultServoY;

  bool initialized_;
  double servo_y_;
  double servo_x_;
};

#endif // DRIVE_HPP