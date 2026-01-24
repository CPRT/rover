#ifndef ARM_HPP
#define ARM_HPP

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class arm : public rclcpp::Node {
public:
  arm();
  void arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_pub;
  control_msgs::msg::JointJog joint_msg;

  void declareParameters();
  void loadParameters();

  // TODO change these constants based on test
  double const maxBaseSpeed = 1.0;
  double const maxWristRollSpeed = 1.0;
  double const maxWristSpeed = 1.0;
  double const maxAct1Speed = 1.0;
  double const maxAct2Speed = 1.0;
  double const maxElbowYawSpeed = 1.0;

  int kThrottleAxis;
  int kBaseAxis;
  int kWristRoll;
  int kWristYaw_positive;
  int kWristYaw_negative;
  int kAct1Axis;
  int kAct2Axis;
  int kElbowYaw;
  int kclaw;
  std::string servoName;
};

#endif