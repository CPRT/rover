#include "arm.hpp"

using MoveToPose = interfaces::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;
using namespace std::placeholders;
arm::arm() : Node("arm_node") {

  declareParameters();
  loadParameters();

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&arm::arm_control, this, std::placeholders::_1));
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  ik_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "servo_node/delta_twist_cmds", 10);
  joint_msg_ = control_msgs::msg::JointJog();
  joint_msg_.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                            "Joint_4", "Joint_5", "Joint_6"};

  this->move_group_action_client_ =
      rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");

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
                           buttons[kWristYaw_positive] -
                               buttons[kWristYaw_negative] * kMaxWristSpeed *
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

void arm::movegroup_goal_response_callback(
    GoalHandleMoveToPose::SharedPtr goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void arm::request_position(int pose_id) {
  geometry_msgs::msg::Pose destination;
  destination.position.x = positions[pose_id][0];
  destination.position.y = positions[pose_id][1];
  destination.position.z = positions[pose_id][2];
  destination.orientation.x = positions[pose_id][3];
  destination.orientation.y = positions[pose_id][4];
  destination.orientation.z = positions[pose_id][5];
  destination.orientation.w = positions[pose_id][6];

  send_goal_pose(destination);
}

void arm::movegroup_feedback_callback(
    GoalHandleMoveToPose::SharedPtr,
    const std::shared_ptr<const MoveToPose::Feedback> feedback) {
  std::stringstream ss;
  ss << "Current position: ";
  for (auto position : feedback->current_position) {
    ss << position << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void arm::movegroup_result_callback(
    const GoalHandleMoveToPose::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  if (result.result->success) {
    RCLCPP_INFO(this->get_logger(), "Move Completed");
  }

  rclcpp::shutdown();
}

void arm::send_goal_pose(geometry_msgs::msg::Pose pose) {
  auto goal_msg = MoveToPose::Goal();
  goal_msg.pose_id = pose;

  RCLCPP_INFO(this->get_logger(), "Sending position");

  auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&arm::movegroup_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&arm::movegroup_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&arm::movegroup_result_callback, this, _1);

  this->move_group_action_client_->async_send_goal(goal_msg, send_goal_options);
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

  // If the code is failing to declare parameters correctly, look here first

  auto overrides =
      this->get_node_parameters_interface()->get_parameter_overrides();
  for (auto const &[key, value] : overrides) {
    if (key.find("saves.") == 0) {
      this->declare_parameter(key, value.get_type());
      RCLCPP_INFO(this->get_logger(), "Manually declared save: %s",
                  key.c_str());
    }
  }
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

  // If the code is failing to load parameters properly, look here second

  auto result =
      this->get_node_parameters_interface()->list_parameters({"saves"}, 0);
  for (auto &name : result.names) {
    positions.push_back(this->get_parameter(name).as_double_array());
    RCLCPP_INFO(this->get_logger(), "Found save: %s", name.c_str());
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}