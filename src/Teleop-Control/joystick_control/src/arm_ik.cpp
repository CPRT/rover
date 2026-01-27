#include "arm_ik.hpp"

arm_ik::arm_ik() : Node("arm_ik_node") {
  declareParameters();
  loadParameters();
  ik_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "servo_node/delta_twist_cmds", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&arm_ik::arm_ik_control, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Arm IK controller started");
}

void arm_ik::arm_ik_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist_msg = geometry_msgs::msg::TwistStamped();

  auto axes = joystickMsg->axes;

  auto buttons = joystickMsg->buttons;

  twist_msg.header = joystickMsg->header;

  //this may or may not be needed, I don't think so though but for now we are leaving it in
  //twist_msg.header.stamp = this->get_clock()->now();

  // this assumes that the twist stamped messages are velocity based. Are they?
  // I'm not sure. Follow up on this.
  // Did someone write this message above me or am I getting dimentia? ^

  twist_msg.twist.linear.x = axes[kForwardAxis] * kMaxForwardSpeed;
  twist_msg.twist.linear.y = axes[kLateralAxis] * kMaxLateralSpeed;
  twist_msg.twist.linear.z = axes[kVerticalAxis] * kMaxVerticalSpeed;
  twist_msg.twist.angular.x = axes[kRollAxis] * kMaxRollSpeed;
  twist_msg.twist.angular.y = axes[kPitchAxis] * kMaxPitchSpeed;
  twist_msg.twist.angular.z = axes[kYawAxis] * kMaxYawSpeed;

  ik_pub_->publish(twist_msg);
}

void arm_ik::declareParameters() {
  this->declare_parameter("forward_axis", 1);
  this->declare_parameter("lateral_axis", 0);
  this->declare_parameter("vertical_axis", 4);
  this->declare_parameter("roll_axis", 3);
  this->declare_parameter("pitch_axis", 2);
  this->declare_parameter("yaw_axis", 5);
  this->declare_parameter("gripper_button", 0);

  this->declare_parameter("max_forward_speed", 0.1);
  this->declare_parameter("max_lateral_speed", 0.1);
  this->declare_parameter("max_vertical_speed", 0.1);
  this->declare_parameter("max_roll_speed", 0.1);
  this->declare_parameter("max_pitch_speed", 0.1);
  this->declare_parameter("max_yaw_speed", 0.1);
}

void arm_ik::loadParameters() {
  this->get_parameter("forward_axis", kForwardAxis);
  this->get_parameter("lateral_axis", kLateralAxis);
  this->get_parameter("vertical_axis", kVerticalAxis);
  this->get_parameter("roll_axis", kRollAxis);
  this->get_parameter("pitch_axis", kPitchAxis);
  this->get_parameter("yaw_axis", kYawAxis);
  this->get_parameter("gripper_button", kGripperButton);

  this->get_parameter("max_forward_speed", kMaxForwardSpeed);
  this->get_parameter("max_lateral_speed", kMaxLateralSpeed);
  this->get_parameter("max_vertical_speed", kMaxVerticalSpeed);
  this->get_parameter("max_roll_speed", kMaxRollSpeed);
  this->get_parameter("max_pitch_speed", kMaxPitchSpeed);
  this->get_parameter("max_yaw_speed", kMaxYawSpeed);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_ik>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}