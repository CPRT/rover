#ifndef DRILL_HPP
#define DRILL_HPP

#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "sensor_msgs/msg/joy.hpp"

class drill : public rclcpp::Node {
public:
  drill();

private:
  void drill_control(std::shared_ptr<sensor_msgs::msg::Joy> joystick_msg);
  void declare_parameters();
  void load_parameters();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr drill_pub_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr elevator_pub_;

  int k_drill_power_axis_;
  int k_drill_elevation_axis_;
  double k_max_drill_duty_;
  double k_max_elevator_duty_;
  std::string k_drill_cmd_topic_;
  std::string k_elevator_cmd_topic_;
};

#endif // DRILL_HPP
