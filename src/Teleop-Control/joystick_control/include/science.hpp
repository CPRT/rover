#ifndef SCIENCE_HPP
#define SCIENCE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int32.hpp"

class science : public rclcpp::Node {
public:
  science();
  void science_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drill_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr test_servo_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void declare_parameters();
  void load_parameters();

  // Parameters
  int kDrillButton;
  int kDrillElevationAxis;
  int kTestServoButton;

  bool initialized_;
};

#endif // SCIENCE_HPP