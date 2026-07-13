#include "arm_teleop.hpp"
#include <algorithm>
#include <condition_variable>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

namespace joystick_control {

ArmTeleop::ArmTeleop(const rclcpp::NodeOptions &options)
    : Node("arm_node", options) {
  declareParameters();
  loadParameters();
  current_state_ = NONE;
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(2).best_effort(),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(mtx_);
          curr_msg_ = msg;
        }
        cv_.notify_one();
      });
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  ik_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
  eef_pub_ = this->create_publisher<ros_phoenix::msg::MotorControl>(
      "/end_effector/set", 10);
  dot_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/rtp_client_node/dot", 10);
  state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "~/state", rclcpp::QoS(1).reliable().transient_local());
  go_to_named_pose_client_ =
      this->create_client<interfaces::srv::GoToNamedPose>(
          "/move_group_client/go_to_named_pose");
  save_current_pose_client_ =
      this->create_client<interfaces::srv::SaveCurrentPose>(
          "/move_group_client/save_current_pose");
  go_to_cam_coord_client_ = this->create_client<interfaces::srv::GoToCamCoord>(
      "/move_group_client/go_to_cam_coord");
  stop_move_group_client_ =
      this->create_client<std_srvs::srv::Trigger>("/move_group_client/stop");
  servo_input_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
      "/servo_node/switch_command_type");
  clear_dot();
  running_ = true;
  run_thread_ = std::make_shared<std::thread>(std::bind(&ArmTeleop::run, this));
  RCLCPP_INFO(this->get_logger(), "Arm controller started");
}

ArmTeleop::~ArmTeleop() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    running_ = false;
  }
  cv_.notify_all();
  if (run_thread_ && run_thread_->joinable()) {
    run_thread_->join();
  }
}

bool ArmTeleop::stop_move_group_motion() {
  if (!stop_move_group_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "MoveGroup stop service is unavailable!!!!");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = stop_move_group_client_->async_send_request(request);

  if (result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timed out stopping MoveGroup motion!!!!");
    return false;
  }

  const auto response = result.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Could not stop MoveGroup motion: %s",
                 response->message.c_str());
    return false;
  }

  return true;
}

bool ArmTeleop::go_to_named_pose(const std::string &name) {
  if (!go_to_named_pose_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "MoveGroup named pose service is unavailable!!!!");
    return false;
  }

  auto request = std::make_shared<interfaces::srv::GoToNamedPose::Request>();
  request->name = name;

  auto result = go_to_named_pose_client_->async_send_request(request);

  if (result.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timed out moving to named pose %s!!!!",
                 name.c_str());
    return false;
  }

  const auto response = result.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Could not move to named pose %s: %s",
                 name.c_str(), response->message.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Moved to named pose %s", name.c_str());
  return true;
}

bool ArmTeleop::save_current_pose(const std::string &name) {
  if (!save_current_pose_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "MoveGroup save pose service is unavailable!!!!");
    return false;
  }

  auto request = std::make_shared<interfaces::srv::SaveCurrentPose::Request>();
  request->name = name;

  auto result = save_current_pose_client_->async_send_request(request);

  if (result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timed out saving pose %s!!!!",
                 name.c_str());
    return false;
  }

  const auto response = result.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Could not save pose %s: %s", name.c_str(),
                 response->message.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Saved current pose as %s", name.c_str());
  return true;
}

bool ArmTeleop::go_to_cam_coord(double u, double v) {
  if (!go_to_cam_coord_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "MoveGroup camera coordinate service is unavailable!!!!");
    return false;
  }

  auto request = std::make_shared<interfaces::srv::GoToCamCoord::Request>();
  request->u = u;
  request->v = v;

  auto result = go_to_cam_coord_client_->async_send_request(request);

  if (result.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timed out moving to camera coordinate (%f, %f)!!!!", u, v);
    return false;
  }

  const auto response = result.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not move to camera coordinate (%f, %f): %s", u, v,
                 response->message.c_str());
    return false;
  }

  return true;
}

void ArmTeleop::endeffector_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto &buttons = joystickMsg->buttons;
  auto eef_control_msg = ros_phoenix::msg::MotorControl();
  eef_control_msg.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  // set the value - close is negative
  double value = 0.0;
  if (buttons[kClawClose]) {
    value = 0.6;
  } else if (buttons[kClawOpen]) {
    value = -0.5;
  }
  eef_control_msg.value = value;
  eef_pub_->publish(eef_control_msg);
}

bool ArmTeleop::moveit_servo_configure(const ArmState requested_state) {
  if (requested_state == NONE) {
    return true;
  }
  if (!servo_input_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Moveit servo service is unavailable!!!!");
    return false;
  }

  auto request =
      std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  std::string type_name = "Unknown";
  if (requested_state == IK) {
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
    type_name = "Twist";
  } else if (requested_state == MANUAL) {
    request->command_type =
        moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
    type_name = "Joint Jog";
  }
  auto result = servo_input_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timed out switching moveit servo type!!!!");
    return false;
  }

  const auto response = result.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Could not switch moveit servo type!!!!");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Switched to moveit servo type %s",
              type_name.c_str());
  return true;
}

std::string ArmTeleop::state_to_string(const ArmState state) {
  switch (state) {
  case ArmState::MANUAL:
    return "Manual";
  case ArmState::IK:
    return "Cartesian IK";
  case ArmState::POS:
    return "Visual Position";
  case ArmState::NONE:
    return "Disabled";
  default:
    return "Unknown";
  }
}

bool ArmTeleop::switch_states(const ArmState requested_state) {
  if (!moveit_servo_configure(requested_state)) {
    return false;
  }

  if (current_state_ == NONE) {
    stop_move_group_motion();
  }

  current_state_ = requested_state;
  auto msg = std_msgs::msg::String();
  msg.data = state_to_string(current_state_);
  state_pub_->publish(msg);
  return true;
}

void ArmTeleop::clear_dot() {
  geometry_msgs::msg::Vector3 msg;
  msg.x = -1;
  msg.y = -1;
  dot_pub_->publish(msg);
  targetPositionX = kCamWidth / 2;
  targetPositionY = kCamHeight / 2;
}

bool ArmTeleop::check_initialized(
    const sensor_msgs::msg::Joy::SharedPtr joystickMsg) {
  if (std::abs(joystickMsg->axes[kJoint1Axis]) < 0.01 &&
      std::abs(joystickMsg->axes[kJoint2Axis]) < 0.01 &&
      std::abs(joystickMsg->axes[kJoint3Axis]) < 0.01 &&
      std::abs(joystickMsg->axes[kJoint4Axis]) < 0.01 &&
      std::abs(joystickMsg->axes[kJoint6Axis]) < 0.01) {
    return true;
  }

  RCLCPP_WARN_THROTTLE(
      this->get_logger(), *(this->get_clock()), 1000,
      "Arm Controller not reading zeros on joystick axes. "
      "Please center joysticks to initialize. (Throttled to 1s)");
  return false;
}

ArmTeleop::ArmState
ArmTeleop::requested_state(const std::vector<int32_t> &buttons) const {
  if (buttons[kDisableButton]) {
    return NONE;
  }

  if (buttons[kIkButton]) {
    return IK;
  }

  if (buttons[kManualButton]) {
    return MANUAL;
  }

  if (buttons[kPositionButton]) {
    return POS;
  }

  return current_state_;
}

void ArmTeleop::run() {
  while (true) {
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg;
    {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_.wait(lock, [this]() { return !running_ || curr_msg_; });
      if (!running_) {
        break;
      }
      joystickMsg = curr_msg_;
      curr_msg_.reset();
    }

    if (!initialized_) {
      initialized_ = check_initialized(joystickMsg);
      last_msg_ = joystickMsg;
      continue;
    }

    const auto requested = requested_state(joystickMsg->buttons);
    const bool state_changed = current_state_ != requested;
    if (state_changed && !switch_states(requested)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to switch states!");
      continue;
    }
    if (current_state_ == IK || current_state_ == MANUAL) {
      endeffector_control(joystickMsg);
    }
    if (state_changed && current_state_ != POS) {
      clear_dot();
    }
    switch (current_state_) {
    case MANUAL:
      manual_arm_control(joystickMsg);
      break;
    case IK:
      ik_arm_control(joystickMsg);
      break;
    case POS:
      ik_pose_control(joystickMsg);
      break;
    default:
      break;
    }
    last_msg_ = joystickMsg;
  }
}

void ArmTeleop::manual_arm_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                           "Joint_4", "Joint_5", "Joint_6"};

  auto &axes = joystickMsg->axes;
  auto &buttons = joystickMsg->buttons;

  joint_msg.header.stamp = joystickMsg->header.stamp;

  joint_msg.velocities = {-axes[kJoint1Axis],
                          -axes[kJoint2Axis],
                          axes[kJoint3Axis],
                          axes[kJoint4Axis],
                          -static_cast<double>(buttons[kWristYaw_positive] -
                                               buttons[kWristYaw_negative]),
                          -axes[kJoint6Axis]};
  // Map throttle from [-1, 1] to [0, 1]
  const auto throttle = (axes[kThrottleAxis] + 1.0) / 2.0;
  for (auto &vel : joint_msg.velocities) {
    vel *= throttle;
  }
  joint_pub_->publish(joint_msg);
}

void ArmTeleop::ik_arm_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = joystickMsg->header.stamp;
  twist_msg.header.frame_id = "EndEffector";

  auto &axes = joystickMsg->axes;
  auto &buttons = joystickMsg->buttons;

  const auto throttle = (axes[kThrottleAxis] + 1.0) / 2.0;

  twist_msg.twist.linear.x = axes[kJoint3Axis] * throttle;
  twist_msg.twist.linear.y = axes[kJoint1Axis] * throttle;
  twist_msg.twist.linear.z = axes[kJoint2Axis] * throttle;
  twist_msg.twist.angular.x = -axes[kJoint4Axis] * throttle;
  twist_msg.twist.angular.y = static_cast<double>(buttons[kWristYaw_positive] -
                                                  buttons[kWristYaw_negative]) *
                              throttle;
  twist_msg.twist.angular.z = -axes[kJoint6Axis] * throttle;

  ik_pub_->publish(twist_msg);
}

void ArmTeleop::ik_pose_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (joystickMsg->buttons[kClawOpen]) {
    stop_move_group_motion();
    return;
  }

  if (!last_msg_) {
    return;
  }

  targetPositionX -= joystickMsg->axes[kJoint6Axis] * kDotInc;
  targetPositionY -= joystickMsg->axes[kJoint3Axis] * kDotInc;
  targetPositionX =
      std::clamp(targetPositionX, 0.0, static_cast<double>(kCamWidth));
  targetPositionY =
      std::clamp(targetPositionY, 0.0, static_cast<double>(kCamHeight));

  bool wasPressed = last_msg_->buttons[kClawClose];
  bool isPressed = joystickMsg->buttons[kClawClose];
  if (wasPressed && !isPressed) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = -1;
    msg.y = -1;
    dot_pub_->publish(msg);
    go_to_cam_coord(targetPositionX, targetPositionY);
    targetPositionX = kCamWidth / 2;
    targetPositionY = kCamHeight / 2;
  }

  clipboards_control(joystickMsg);
  geometry_msgs::msg::Vector3 msg;
  msg.x = targetPositionX;
  msg.y = targetPositionY;
  dot_pub_->publish(msg);
}

void ArmTeleop::clipboards_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  bool isPressed = joystickMsg->buttons[kClipboard1SaveButton];
  bool wasPressed = last_msg_->buttons[kClipboard1SaveButton];
  if (wasPressed && !isPressed) {
    if (save_current_pose("clipboard_1")) {
      RCLCPP_INFO(this->get_logger(), "Saved current pose to clipboard 1");
    }
  }

  isPressed = joystickMsg->buttons[kClipboard1ExecuteButton];
  wasPressed = last_msg_->buttons[kClipboard1ExecuteButton];
  if (wasPressed && !isPressed) {
    if (go_to_named_pose("clipboard_1")) {
      RCLCPP_INFO(this->get_logger(), "Executed pose from clipboard 1");
    }
  }

  isPressed = joystickMsg->buttons[kClipboard2SaveButton];
  wasPressed = last_msg_->buttons[kClipboard2SaveButton];
  if (wasPressed && !isPressed) {
    if (save_current_pose("clipboard_2")) {
      RCLCPP_INFO(this->get_logger(), "Saved current pose to clipboard 2");
    }
  }

  isPressed = joystickMsg->buttons[kClipboard2ExecuteButton];
  wasPressed = last_msg_->buttons[kClipboard2ExecuteButton];
  if (wasPressed && !isPressed) {
    if (go_to_named_pose("clipboard_2")) {
      RCLCPP_INFO(this->get_logger(), "Executed pose from clipboard 2");
    }
  }
}

void ArmTeleop::declareParameters() {
  this->declare_parameter("throttle.axis", 2);
  this->declare_parameter("joint_1_axis", 0);
  this->declare_parameter("joint_2_axis", 1);
  this->declare_parameter("joint_3_axis", 5);
  this->declare_parameter("joint_4_axis", 3);
  this->declare_parameter("joint_6_axis", 4);
  this->declare_parameter("wrist_yaw_positive", 3);
  this->declare_parameter("wrist_yaw_negative", 5);
  this->declare_parameter("ik_button", 10);
  this->declare_parameter("manual_button", 11);
  this->declare_parameter("position_button", 8);
  this->declare_parameter("disable_button", 9);
  this->declare_parameter("claw_close_button", 0);
  this->declare_parameter("claw_open_button", 1);
  this->declare_parameter("cam_width", 1920);
  this->declare_parameter("cam_height", 1080);
  this->declare_parameter("dot_inc", 2);
  this->declare_parameter("clipboard1.save_button", 2);
  this->declare_parameter("clipboard1.execute_button", 4);
  this->declare_parameter("clipboard2.save_button", 3);
  this->declare_parameter("clipboard2.execute_button", 5);
}

void ArmTeleop::loadParameters() {
  this->get_parameter("throttle.axis", kThrottleAxis);
  this->get_parameter("joint_1_axis", kJoint1Axis);
  this->get_parameter("joint_2_axis", kJoint2Axis);
  this->get_parameter("joint_3_axis", kJoint3Axis);
  this->get_parameter("joint_4_axis", kJoint4Axis);
  this->get_parameter("joint_6_axis", kJoint6Axis);
  this->get_parameter("wrist_yaw_positive", kWristYaw_positive);
  this->get_parameter("wrist_yaw_negative", kWristYaw_negative);
  this->get_parameter("ik_button", kIkButton);
  this->get_parameter("manual_button", kManualButton);
  this->get_parameter("position_button", kPositionButton);
  this->get_parameter("disable_button", kDisableButton);
  this->get_parameter("claw_close_button", kClawClose);
  this->get_parameter("claw_open_button", kClawOpen);
  this->get_parameter("cam_width", kCamWidth);
  this->get_parameter("cam_height", kCamHeight);
  this->get_parameter("dot_inc", kDotInc);
  this->get_parameter("clipboard1.save_button", kClipboard1SaveButton);
  this->get_parameter("clipboard1.execute_button", kClipboard1ExecuteButton);
  this->get_parameter("clipboard2.save_button", kClipboard2SaveButton);
  this->get_parameter("clipboard2.execute_button", kClipboard2ExecuteButton);
}

} // namespace joystick_control

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_control::ArmTeleop)