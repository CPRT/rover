#ifndef DRILL_HPP
#define DRILL_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class drill : public rclcpp::Node {
public:
  drill();

private:
  void drill_control(std::shared_ptr<sensor_msgs::msg::Joy> joystick_msg);
  void declare_parameters();
  void load_parameters();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drill_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_pub_;
  rclcpp::TimerBase::SharedPtr joy_watchdog_timer_;

  void on_joy_watchdog();
  void mark_joy_received();

  int k_drill_power_axis_;
  int k_drill_elevation_axis_;
  double k_max_drill_duty_;
  double k_max_elevator_duty_;
  double k_joy_first_message_timeout_s_;
  std::string k_drill_cmd_topic_;
  std::string k_elevator_cmd_topic_;

  bool joy_received_{false};
};

#endif // DRILL_HPP
