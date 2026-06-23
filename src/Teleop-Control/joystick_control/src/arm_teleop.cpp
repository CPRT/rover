#include "arm_teleop.hpp"
#include "std_srvs/srv/trigger.hpp"
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
  name_service_ = this->create_service<interfaces::srv::GetPoses>(
      "get_names_poses",
      [this](const std::shared_ptr<interfaces::srv::GetPoses::Request>,
             std::shared_ptr<interfaces::srv::GetPoses::Response> response) {
        response->pose_names = moveit_client_.getNamedTargets();
      });
  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_motion",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        moveit_client_.stop();
        response->success = true;
        response->message = "Motion stopped";
      });
  go_to_pose_service_ = this->create_service<interfaces::srv::GoToPose>(
      "go_to_pose",
      [this](const std::shared_ptr<interfaces::srv::GoToPose::Request> request,
             std::shared_ptr<interfaces::srv::GoToPose::Response> response) {
        if (current_state_ != NONE) {
          response->success = false;
          response->message = "Joystick is active";
          RCLCPP_WARN(this->get_logger(),
                      "Could not move to pose %s: Joystick is active",
                      request->name.c_str());
          return;
        }
        response->success = moveit_client_.goToPose(request->name);
        response->message = response->success
                                ? "Planning succeeded..."
                                : "Failed to plan to pose: " + request->name;
      });
  switch_client_ =
      this->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");
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

bool ArmTeleop::switch_states(const ArmState requested_state) {
  constexpr char const *servo_contoller = "arm_controller_servo";
  constexpr char const *move_group_contoller = "arm_controller_move_group";

  if (current_state_ == requested_state) {
    return true;
  }
  bool servo_needed = requested_state == IK || requested_state == MANUAL;
  bool servo_on = current_state_ == IK || current_state_ == MANUAL;

  if (servo_needed == servo_on) {
    current_state_ = requested_state;
    return true;
  }
  std::string wanted_controller =
      servo_needed ? servo_contoller : move_group_contoller;
  std::string current_controller =
      servo_on ? servo_contoller : move_group_contoller;
  if (current_controller == wanted_controller) {
    // Logic above should mean this never happens
    RCLCPP_WARN(this->get_logger(),
                "(%s:%d) Current controller is the same as wanted controller "
                "despite ealier check",
                __FILE__, __LINE__);
    return false;
  }
  // Switch controller
  if (!switch_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Controller switch service is unavailable!!!!");
    return false;
  }

  auto switch_request = std::make_shared<
      controller_manager_msgs::srv::SwitchController::Request>();
  switch_request->activate_controllers.push_back(wanted_controller);
  switch_request->deactivate_controllers.push_back(current_controller);
  switch_request->strictness =
      controller_manager_msgs::srv::SwitchController::Request::STRICT;
  switch_request->activate_asap = true;

  auto switch_result = switch_client_->async_send_request(switch_request);
  if (switch_result.wait_for(std::chrono::seconds(5)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timed out switching ros2_control controller!!!!");
    return false;
  }
  const auto switch_response = switch_result.get();
  if (!switch_response->ok) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not switch ros2_control controller!!!!");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Switched to ros2_control controller %s",
              wanted_controller.c_str());

  // Enable/disable moveit servo
  // TODO: Determind if this is actually necessary or if the controllers can
  // listen on different topics avoiding the conflict
  std::string moveit_service_name =
      servo_needed ? "/servo_node/start_servo" : "/servo_node/stop_servo";
  auto moveit_client =
      this->create_client<std_srvs::srv::Trigger>(moveit_service_name.c_str());

  if (!moveit_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not find service to %s Moveit Servo!!!!",
                 servo_needed ? "enable" : "disable");
    return false;
  }

  auto servo_result = moveit_client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  if (servo_result.wait_for(std::chrono::seconds(5)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timed out trying to %s Moveit Servo!!!!",
                 servo_needed ? "enable" : "disable");
    return false;
  }
  const auto servo_response = servo_result.get();
  if (!servo_response->success) {
    RCLCPP_ERROR(this->get_logger(), "Could not %s Moveit Servo: %s",
                 servo_needed ? "enable" : "disable",
                 servo_response->message.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), " %s Moveit Servo",
              servo_needed ? "enabled" : "disabled");
  if (current_state_ == NONE) {
    moveit_client_.stop();
  }

  current_state_ = requested_state;
  return true;
}

bool ArmTeleop::start_moveit_servo() {
  auto moveit_client =
      this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

  if (!moveit_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not find service to enable Moveit Servo!!!!");
    return false;
  }

  auto servo_result = moveit_client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  if (servo_result.wait_for(std::chrono::seconds(5)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timed out trying to enable Moveit Servo!!!!");
    return false;
  }
  const auto servo_response = servo_result.get();
  if (!servo_response->success) {
    RCLCPP_ERROR(this->get_logger(), "Could not enable Moveit Servo: %s",
                 servo_response->message.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Moveit Servo enabled");
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
    return start_moveit_servo();
  }
  RCLCPP_WARN_THROTTLE(
      this->get_logger(), *(this->get_clock()), 1,
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

  joint_pub_->publish(joint_msg);
}

void ArmTeleop::ik_arm_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = joystickMsg->header.stamp;
  twist_msg.header.frame_id = "EndEffector";

  auto &axes = joystickMsg->axes;
  auto &buttons = joystickMsg->buttons;

  twist_msg.twist.linear.x = axes[kJoint2Axis];
  twist_msg.twist.linear.y = axes[kJoint1Axis];
  twist_msg.twist.linear.z = axes[kJoint3Axis];
  twist_msg.twist.angular.x = axes[kJoint4Axis];
  twist_msg.twist.angular.y = axes[kJoint6Axis];
  twist_msg.twist.angular.z =
      (static_cast<double>(buttons[kWristYaw_positive] -
                           buttons[kWristYaw_negative])) *
      axes[kThrottleAxis];

  ik_pub_->publish(twist_msg);
}

void ArmTeleop::ik_pose_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (joystickMsg->buttons[kClawOpen]) {
    moveit_client_.stop();
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
    moveit_client_.goToCamCoord(targetPositionX, targetPositionY);
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
    clipboard1_pose_index_ = moveit_client_.saveCurrentPose();
    RCLCPP_INFO(this->get_logger(),
                "Saved current pose to clipboard 1 at index %d",
                clipboard1_pose_index_);
  }
  isPressed = joystickMsg->buttons[kClipboard1ExecuteButton];
  wasPressed = last_msg_->buttons[kClipboard1ExecuteButton];
  if (wasPressed && !isPressed && clipboard1_pose_index_ != -1) {
    moveit_client_.goToSavedPose(clipboard1_pose_index_);
    RCLCPP_INFO(this->get_logger(),
                "Executed pose from clipboard 1 at index %d",
                clipboard1_pose_index_);
  }
  isPressed = joystickMsg->buttons[kClipboard2SaveButton];
  wasPressed = last_msg_->buttons[kClipboard2SaveButton];
  if (wasPressed && !isPressed) {
    clipboard2_pose_index_ = moveit_client_.saveCurrentPose();
    RCLCPP_INFO(this->get_logger(),
                "Saved current pose to clipboard 2 at index %d",
                clipboard2_pose_index_);
  }
  isPressed = joystickMsg->buttons[kClipboard2ExecuteButton];
  wasPressed = last_msg_->buttons[kClipboard2ExecuteButton];
  if (wasPressed && !isPressed && clipboard2_pose_index_ != -1) {
    moveit_client_.goToSavedPose(clipboard2_pose_index_);
    RCLCPP_INFO(this->get_logger(),
                "Executed pose from clipboard 2 at index %d",
                clipboard2_pose_index_);
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