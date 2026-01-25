#include "arm.hpp"
#include "ArmHelpers.hpp"

arm::arm() : Node("arm_node") {

  declareParameters();
  loadParameters();

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/controller_b/joy", 10,
      std::bind(&arm::arm_control, this, std::placeholders::_1));
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  if (!ArmHelpers::start_moveit_servo(this)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start MoveIt servo service, Manual mode will not work");
  }
  joint_msg_ = control_msgs::msg::JointJog();
  joint_msg_.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                           "Joint_4", "Joint_5", "Joint_6"};

  servo_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/" + servoName, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  RCLCPP_INFO(this->get_logger(), "Arm controller started");
}

void arm::arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto joint_msg_ = control_msgs::msg::JointJog();

  auto axes = joystickMsg->axes;
  auto buttons = joystickMsg->buttons;

  joint_msg_.header = joystickMsg->header;

  joint_msg_.velocities = {axes[kBaseAxis] * maxBaseSpeed,
                          axes[kAct1Axis] * maxAct1Speed,
                          axes[kAct2Axis] * maxAct2Speed,
                          axes[kWristRoll] * maxWristRollSpeed,
                          axes[kElbowYaw] * maxElbowYawSpeed,
                          buttons[kWristYaw_positive] -
                              buttons[kWristYaw_negative] * maxWristSpeed *
                                  axes[kThrottleAxis]};

  joint_pub_->publish(joint_msg_);

  if (joystickMsg->buttons[kClaw] == 1) {
    auto servo_msg = std_msgs::msg::Float32();
    servo_msg.data = 1.0; 
    servo_pub_->publish(servo_msg);
  } else {
    auto servo_msg = std_msgs::msg::Float32();
    servo_msg.data = 0.0;
    servo_pub_->publish(servo_msg);
  }
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
  this->declare_parameter("claw", 1);
  this->declare_parameter("servo_name", "arm");
  this->declare_parameter("max_base_speed", 1.0);
  this->declare_parameter("max_wrist_roll_speed", 1.0);
  this->declare_parameter("max_wrist_speed", 1.0);
  this->declare_parameter("max_act1_speed", 1.0);
  this->declare_parameter("max_act2_speed", 1.0);
  this->declare_parameter("max_elbow_yaw_speed", 1.0);
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
  this->get_parameter("claw", kClaw);
  this->get_parameter("servo_name", servoName);
  this->get_parameter("max_base_speed", kMaxBaseSpeed);
  this->get_parameter("max_wrist_roll_speed", kMaxWristRollSpeed);
  this->get_parameter("max_wrist_speed", kMaxWristSpeed);
  this->get_parameter("max_act1_speed", kMaxAct1Speed);
  this->get_parameter("max_act2_speed", kMaxAct2Speed);
  this->get_parameter("max_elbow_yaw_speed", kMaxElbowYawSpeed);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}