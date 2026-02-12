#include "arm.hpp"

arm::arm() : Node("arm_node") {

  declareParameters();
  loadParameters();

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&arm::arm_control, this, std::placeholders::_1));
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);Joy
  ik_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "servo_node/delta_twist_cmds", 10);
  joint_msg_ = control_msgs::msg::JointJog();
  joint_msg_.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                            "Joint_4", "Joint_5", "Joint_6"};

  RCLCPP_INFO(this->get_logger(), "Arm controller started");
}

void arm::arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  mode_switch(joystickMsg);
  if (mode == 1) {
    arm_manual(joystickMsg);
  }
  if (mode == 2) {
    arm_ik(joystickMsg);
  }
}

void arm::arm_manual(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto joint_msg_ = control_msgs::msg::JointJog();

  auto axes = joystickMsg->axes;
  auto buttons = joystickMsg->buttons;

  joint_msg_.header = joystickMsg->header;

  joint_msg_.velocities = {axes[kBaseAxis] * kMaxBaseSpeed,
                           axes[kAct1Axis] * kMaxAct1Speed,
                           axes[kAct2Axis] * kMaxAct2Speed,
                           axes[kWristRoll] * kMaxWristRollSpeed,
                           axes[kElbowYaw] * kMaxElbowYawSpeed,
                           (buttons[kWristYaw_positive] -
                               buttons[kWristYaw_negative]) * kMaxWristSpeed *
                                   axes[kThrottleAxis]};

  joint_pub_->publish(joint_msg_);
}

void arm::mode_switch(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  mode_button_value = joystickMsg->buttons[mode_switch_button];

  if ((mode_button_value == 0) && (mode_button_value_prev == 1)) {
    mode = mode + 1;
    // This logic is here to allow us to add more modes like science perhaps
    if (mode > modes) { // Modes is the number of modes we have
      mode = 1;
    }
  }

  mode_button_value_prev = joystickMsg->buttons[mode_switch_button];
}

void arm::arm_ik(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist_msg = geometry_msgs::msg::TwistStamped();

  auto axes = joystickMsg->axes;

  auto buttons = joystickMsg->buttons;

  twist_msg.header = joystickMsg->header;

  twist_msg.twist.linear.x = axes[kForwardAxis] * kMaxForwardSpeed;
  twist_msg.twist.linear.y = axes[kLateralAxis] * kMaxLateralSpeed;
  twist_msg.twist.linear.z = axes[kVerticalAxis] * kMaxVerticalSpeed;
  twist_msg.twist.angular.x = axes[kRollAxis] * kMaxRollSpeed;
  twist_msg.twist.angular.y = axes[kPitchAxis] * kMaxPitchSpeed;
  twist_msg.twist.angular.z = axes[kYawAxis] * kMaxYawSpeed;

  ik_pub_->publish(twist_msg);
}
void arm::declareParameters() {
  this->declare_parameter("arm_manual.throttle.axis", 2);
  this->declare_parameter("arm_manual.base_axis", 0);
  this->declare_parameter("arm_manual.wrist_roll", 4);
  this->declare_parameter("arm_manual.wrist_yaw_positive", 3);
  this->declare_parameter("arm_manual.wrist_yaw_negative", 5);
  this->declare_parameter("arm_manual.act1_axis", 1);
  this->declare_parameter("arm_manual.act2_axis", 5);
  this->declare_parameter("arm_manual.elbow_yaw", 6);
  this->declare_parameter("arm_manual.max_base_speed", 1.0);
  this->declare_parameter("arm_manual.max_wrist_roll_speed", 1.0);
  this->declare_parameter("arm_manual.max_wrist_speed", 1.0);
  this->declare_parameter("arm_manual.max_act1_speed", 1.0);
  this->declare_parameter("arm_manual.max_act2_speed", 1.0);
  this->declare_parameter("arm_manual.max_elbow_yaw_speed", 1.0);

  this->declare_parameter("arm_ik.forward_axis", 1);
  this->declare_parameter("arm_ik.lateral_axis", 0);
  this->declare_parameter("arm_ik.vertical_axis", 4);
  this->declare_parameter("arm_ik.roll_axis", 3);
  this->declare_parameter("arm_ik.pitch_axis", 2);
  this->declare_parameter("arm_ik.yaw_axis", 5);
  this->declare_parameter("arm_ik.gripper_button", 0);
  this->declare_parameter("arm_ik.max_forward_speed", 0.1);
  this->declare_parameter("arm_ik.max_lateral_speed", 0.1);
  this->declare_parameter("arm_ik.max_vertical_speed", 0.1);
  this->declare_parameter("arm_ik.max_roll_speed", 0.1);
  this->declare_parameter("arm_ik.max_pitch_speed", 0.1);
  this->declare_parameter("arm_ik.max_yaw_speed", 0.1);

  this->declare_parameter("mode_switch_button", 7);
}
void arm::loadParameters() {
  this->get_parameter("arm_manual.throttle.axis", kThrottleAxis);
  this->get_parameter("arm_manual.base_axis", kBaseAxis);
  this->get_parameter("arm_manual.wrist_roll", kWristRoll);
  this->get_parameter("arm_manual.wrist_yaw_positive", kWristYaw_positive);
  this->get_parameter("arm_manual.wrist_yaw_negative", kWristYaw_negative);
  this->get_parameter("arm_manual.act1_axis", kAct1Axis);
  this->get_parameter("arm_manual.act2_axis", kAct2Axis);
  this->get_parameter("arm_manual.elbow_yaw", kElbowYaw);
  this->get_parameter("arm_manual.max_base_speed", kMaxBaseSpeed);
  this->get_parameter("arm_manual.max_wrist_roll_speed", kMaxWristRollSpeed);
  this->get_parameter("arm_manual.max_wrist_speed", kMaxWristSpeed);
  this->get_parameter("arm_manual.max_act1_speed", kMaxAct1Speed);
  this->get_parameter("arm_manual.max_act2_speed", kMaxAct2Speed);
  this->get_parameter("arm_manual.max_elbow_yaw_speed", kMaxElbowYawSpeed);

  this->get_parameter("arm_ik.forward_axis", kForwardAxis);
  this->get_parameter("arm_ik.lateral_axis", kLateralAxis);
  this->get_parameter("arm_ik.vertical_axis", kVerticalAxis);
  this->get_parameter("arm_ik.roll_axis", kRollAxis);
  this->get_parameter("arm_ik.pitch_axis", kPitchAxis);
  this->get_parameter("arm_ik.yaw_axis", kYawAxis);
  this->get_parameter("arm_ik.gripper_button", kGripperButton);
  this->get_parameter("arm_ik.max_forward_speed", kMaxForwardSpeed);
  this->get_parameter("arm_ik.max_lateral_speed", kMaxLateralSpeed);
  this->get_parameter("arm_ik.max_vertical_speed", kMaxVerticalSpeed);
  this->get_parameter("arm_ik.max_roll_speed", kMaxRollSpeed);
  this->get_parameter("arm_ik.max_pitch_speed", kMaxPitchSpeed);
  this->get_parameter("arm_ik.max_yaw_speed", kMaxYawSpeed);

  this->get_parameter("mode_switch_button", mode_switch_button);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}