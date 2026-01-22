#ifndef ARM_HPP
#define ARM_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class arm : public rclcpp::Node {
public:
  arm();
  void arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub;
};