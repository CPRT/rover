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

  int artificial_wrist_axis(int, int);

  declareParameters();
  loadParameters();

  double const baseSpeed = 1.0;
  double const wristRollSpeed = 1.0;
  double const wristSpeed = 1.0;
  double const act1Speed = 1.0;
  double const act2Speed = 1.0;
  double const elbowYawSpeed = 1.0;

  int kThrottleAxis;
  int kBaseAxis;
  int kWristRoll;
  int kWristYaw;
  int kAct1Axis;
  int kAct2Axis;
  int kElbowYaw;
  int kclaw;
  
};