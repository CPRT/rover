#ifndef ARM_IK_HPP
#define ARM_IK_HPP

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class arm_ik : public rclcpp::Node {
public:
  arm_ik();
  void arm_ik_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ik_pub_;

  void declareParameters();
  void loadParameters();

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

#endif // ARM_IK_HPP