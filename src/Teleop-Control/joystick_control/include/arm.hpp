#ifndef ARM_HPP
#define ARM_HPP

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ik_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  control_msgs::msg::JointJog joint_msg_;

  void arm_manual(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void arm_ik(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void mode_switch(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

  int mode_button_value = 0;
  int mode_button_value_prev = 0;
  int modes = 2;
  int mode = 1;

  void declareParameters();
  void loadParameters();

  int mode_switch_button;

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

  int kForwardAxis;
  int kLateralAxis;
  int kVerticalAxis;
  int kRollAxis;
  int kPitchAxis;
  int kYawAxis;

  double kMaxForwardSpeed;
  double kMaxLateralSpeed;
  double kMaxVerticalSpeed;
  double kMaxRollSpeed;
  double kMaxPitchSpeed;
  double kMaxYawSpeed;

  int kGripperButton;
};

#endif