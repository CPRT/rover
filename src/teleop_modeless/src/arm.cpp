#include "arm.hpp"

arm::arm() : Node("arm_node") {
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/controller_b/joy", 10, std::bind(&arm::arm_control, this, std::placeholders::_1));
  joint_pub = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  if (!ArmHelpers::start_moveit_servo(this)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start MoveIt servo service, Manual mode will not work");
  }
  joint_msg_ = control_msgs::msg::JointJog();
  joint_msg_.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                            "Joint_4", "Joint_5", "Joint_6"};

  servo_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
      "/" + servoName, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
}

arm::arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg_.header = joystickMsg->header;

  //sample control logic

  joint_msg.velocities = {joystickMsg->axes[kBaseAxis] * baseSpeed * axes[ kThrottleAxis],
                          joystickMsg->axes[kAct1Axis] * act1Speed * axes[ kThrottleAxis],
                          joystickMsg->axes[kAct2Axis] * act2Speed * axes[ kThrottleAxis],
                          joystickMsg->axes[kWristRoll] * wristRollSpeed * axes[ kThrottleAxis],
                          joystickMsg->axes[kElbowYaw] * elbowYawSpeed * axes[ kThrottleAxis],
                          joystickMsg->artificial_wrist_axis(axes[kWristYaw_positive], axes[kWristYaw_negative]) * wristSpeed * axes[ kThrottleAxis]};

  joint_pub->publish(joint_msg);

  if (joystickMsg->buttons[kclaw] == 1) {
    auto servo_msg = std_msgs::msg::Float32();
    servo_msg.data = 1.0; // Example position
    servo_pub_->publish(servo_msg);
  } else {
    auto servo_msg = std_msgs::msg::Float32();
    servo_msg.data = 0.0; // Example position
    servo_pub_->publish(servo_msg);
  }
}

arm::artificial_wrist_axis(int axis1, int axis2) {
  // Creates a cool artificial axis
  return axis1 - axis2;
}

declareParameters() {
  this->declare_parameter("throttle.axis", 2);
  this->declare_parameter("base_axis", 0);
  this->declare_parameter("wrist_roll", 4);
  this->declare_parameter("wrist_yaw_positive", 3);
  this->declare_parameter("wrist_yaw_negative", 5);
  this->declare_parameter("act1_axis", 1);
  this->declare_parameter("act2_axis", 5);
  this->declare_parameter("elbow_yaw", 6);
  this->declare_parameter("claw", 1);
}
loadParameters() {
  this->get_parameter("throttle.axis", kThrottleAxis);
  this->get_parameter("base_axis", kBaseAxis);
  this->get_parameter("wrist_roll", kWristRoll);
  this->get_parameter("wrist_yaw_positive", kWristYaw_positive);
  this->get_parameter("wrist_yaw_negative", kWristYaw_negative);
  this->get_parameter("act1_axis", kAct1Axis);
  this->get_parameter("act2_axis", kAct2Axis);
  this->get_parameter("elbow_yaw", kElbowYaw);
  this->get_parameter("claw", kclaw);
}