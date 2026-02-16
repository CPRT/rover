#include "arm.hpp"

arm::arm() : Node("arm_node") {

  declareParameters();
  loadParameters();
  current_state_ = NONE;

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&arm::arm_control, this, std::placeholders::_1));
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  ik_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/servo_node/delta_twist_cmds", 10);

  RCLCPP_INFO(this->get_logger(), "Arm controller started");
}

void arm::arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto &buttons = joystickMsg->buttons;
  if (!initialized_) {
    if (std::abs(joystickMsg->axes[kBaseAxis]) < 0.01 &&
        std::abs(joystickMsg->axes[kAct1Axis]) < 0.01 &&
        std::abs(joystickMsg->axes[kAct2Axis]) < 0.01) {
      initialized_ = true;
    }
    return;
  }

  if (buttons[kDisableButton]) {
    current_state_ = NONE;
    RCLCPP_INFO(this->get_logger(), "Arm disabled");
    return;
  } else if (buttons[kIkButton]) {
    current_state_ = IK;
    RCLCPP_INFO(this->get_logger(), "Switched to IK control");
  } else if (buttons[kManualButton]) {
    current_state_ = MANUAL;
    RCLCPP_INFO(this->get_logger(), "Switched to manual control");
  }

  switch (current_state_) {
  case MANUAL:
    manual_arm_control(joystickMsg);
    break;
  case IK:
    ik_arm_control(joystickMsg);
    break;
  default:
    break;
  }
}

void arm::manual_arm_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                           "Joint_4", "Joint_5", "Joint_6"};

  auto &axes = joystickMsg->axes;
  auto &buttons = joystickMsg->buttons;

  joint_msg.header = joystickMsg->header;

  joint_msg.velocities = {
      axes[kBaseAxis],
      axes[kAct1Axis],
      axes[kAct2Axis],
      axes[kWristRoll],
      axes[kElbowYaw],
      buttons[kWristYaw_positive] - buttons[kWristYaw_negative]};

  joint_pub_->publish(joint_msg);
}

void arm::ik_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist_msg = geometry_msgs::msg::Twist();

  auto &axes = joystickMsg->axes;
  auto &buttons = joystickMsg->buttons;

  twist_msg.linear.x = axes[kBaseAxis];
  twist_msg.linear.y = axes[kAct1Axis];
  twist_msg.linear.z = axes[kAct2Axis];
  twist_msg.angular.x = axes[kWristRoll];
  twist_msg.angular.y = axes[kElbowYaw];
  twist_msg.angular.z = (static_cast<double>(buttons[kWristYaw_positive] -
                                             buttons[kWristYaw_negative])) *
                        axes[kThrottleAxis];

  ik_pub_->publish(twist_msg);
}

void arm::declareParameters() {
  this->declare_parameter("throttle.axis", 2);
  this->declare_parameter("base_axis", 0);
  this->declare_parameter("wrist_roll", 4);
  this->declare_parameter("wrist_yaw_positive", 3);
  this->declare_parameter("wrist_yaw_negative", 5);
  this->declare_parameter("act1_axis", 1);
  this->declare_parameter("act2_axis", 5);
  this->declare_parameter("elbow_yaw", 6);
  this->declare_parameter("ik_button", 10);
  this->declare_parameter("manual_button", 11);
  this->declare_parameter("disable_button", 9);
}
void arm::loadParameters() {
  this->get_parameter("throttle.axis", kThrottleAxis);
  this->get_parameter("base_axis", kBaseAxis);
  this->get_parameter("wrist_roll", kWristRoll);
  this->get_parameter("wrist_yaw_positive", kWristYaw_positive);
  this->get_parameter("wrist_yaw_negative", kWristYaw_negative);
  this->get_parameter("act1_axis", kAct1Axis);
  this->get_parameter("act2_axis", kAct2Axis);
  this->get_parameter("elbow_yaw", kElbowYaw);
  this->get_parameter("ik_button", kIkButton);
  this->get_parameter("manual_button", kManualButton);
  this->get_parameter("disable_button", kDisableButton);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}