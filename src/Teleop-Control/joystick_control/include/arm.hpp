#ifndef ARM_HPP
#define ARM_HPP

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class arm : public rclcpp::Node {
public:
  arm();

private:
  void arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void manual_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void ik_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void endeffector_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  bool moveit_servo_state(bool enable);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ik_pub_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr eef_pub_;
  enum ArmState { NONE = 0, MANUAL, IK };

  ArmState current_state_;

  void declareParameters();
  void loadParameters();
  bool initialized_ = false;

  int kThrottleAxis;
  int kJoint1Axis;
  int kJoint2Axis;
  int kJoint3Axis;
  int kJoint4Axis;
  int kJoint6Axis;
  int kWristYaw_positive;
  int kWristYaw_negative;
  int kDisableButton;
  int kIkButton;
  int kManualButton;
  int kClawOpen;
  int kClawClose;
};

#endif