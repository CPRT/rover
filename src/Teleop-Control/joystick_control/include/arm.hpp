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
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  control_msgs::msg::JointJog joint_msg_;

  void declareParameters();
  void loadParameters();

  double kMaxBaseSpeed;
  double kMaxWristRollSpeed;
  double kMaxWristSpeed;
  double kMaxAct1Speed;
  double kMaxAct2Speed;
  double kMaxElbowYawSpeed;
  int kThrottleAxis;
  int kBaseAxis;
  int kWristRoll;
  int kWristYaw_positive;
  int kWristYaw_negative;
  int kAct1Axis;
  int kAct2Axis;
  int kElbowYaw;
};

#endif