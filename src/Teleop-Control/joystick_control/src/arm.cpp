#include "arm.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_srvs/srv/trigger.hpp"

arm::arm() : Node("arm_node") {

  declareParameters();
  loadParameters();
  current_state_ = NONE;

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&arm::arm_control, this, std::placeholders::_1));
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  ik_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
  eef_pub_ = this->create_publisher<ros_phoenix::msg::MotorControl>(
      "/end_effector/set", 10);

  RCLCPP_INFO(this->get_logger(), "Arm controller started");
}
void arm::endeffector_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto &buttons = joystickMsg->buttons;
  auto eef_control_msg = ros_phoenix::msg::MotorControl();
  eef_control_msg.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  // set the value - close is negative
  double value = 0.0;
  if (buttons[kClawClose]) {
    value = -0.6;
  } else if (buttons[kClawOpen]) {
    value = 0.5;
  }
  eef_control_msg.value = value;
  eef_pub_->publish(eef_control_msg);
}

bool arm::moveit_servo_state(bool enable) {
  constexpr int attempts = 3;

  auto switch_client =
      this->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");
  auto switch_request = std::make_shared<
      controller_manager_msgs::srv::SwitchController::Request>();
  if (enable) {
    switch_request->start_controllers.push_back("arm_controller_servo");
    switch_request->stop_controllers.push_back("arm_controller_move_group");
  } else {
    switch_request->start_controllers.push_back("arm_controller_move_group");
    switch_request->stop_controllers.push_back("arm_controller_servo");
  }
  switch_request->strictness =
      controller_manager_msgs::srv::SwitchController::Request::STRICT;
  switch_request->start_asap = true;
  auto future_result = switch_client->async_send_request(
      switch_request,
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::
                 SharedFuture future) {
        try {
          auto result = future.get();
          if (result->ok) {
            RCLCPP_INFO(this->get_logger(),
                        "Controller switched successfully!");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Controller switch failed!");
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
      });

  std::string moveit_service_name =
      enable ? "/servo_node/start_servo" : "/servo_node/stop_servo";
  if (enable) {
    RCLCPP_INFO(this->get_logger(), "Enabling MoveIt Servo");
  } else {
    RCLCPP_INFO(this->get_logger(), "Disabling MoveIt Servo");
  }
  auto moveit_client =
      this->create_client<std_srvs::srv::Trigger>(moveit_service_name.c_str());
  int i = 0;
  while (!moveit_client->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(this->get_logger(), "Service not available, Waiting...");
    if (++i >= attempts) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service not available after %d attempts. Giving up", i);
      return false;
    }
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  moveit_client->async_send_request(request);
  return true;
}

void arm::arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto &buttons = joystickMsg->buttons;
  if (!initialized_) {
    if (std::abs(joystickMsg->axes[kJoint1Axis]) < 0.01 &&
        std::abs(joystickMsg->axes[kJoint2Axis]) < 0.01 &&
        std::abs(joystickMsg->axes[kJoint3Axis]) < 0.01 &&
        std::abs(joystickMsg->axes[kJoint4Axis]) < 0.01 &&
        std::abs(joystickMsg->axes[kJoint6Axis]) < 0.01) {
      initialized_ = true;
    }
    return;
  }

  if (buttons[kDisableButton] && current_state_ != NONE) {
    current_state_ = NONE;
    moveit_servo_state(false);
    RCLCPP_INFO(this->get_logger(), "Arm disabled");
    return;
  } else if (buttons[kIkButton] && current_state_ != IK) {
    current_state_ = IK;
    moveit_servo_state(true);
    RCLCPP_INFO(this->get_logger(), "Switched to IK control");
  } else if (buttons[kManualButton] && current_state_ != MANUAL) {
    current_state_ = MANUAL;
    moveit_servo_state(true);
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

  joint_msg.header.stamp = joystickMsg->header.stamp;

  joint_msg.velocities = {axes[kJoint1Axis],
                          axes[kJoint2Axis],
                          axes[kJoint3Axis],
                          axes[kJoint4Axis],
                          static_cast<double>(buttons[kWristYaw_positive] -
                                              buttons[kWristYaw_negative]),
                          axes[kJoint6Axis]};

  joint_pub_->publish(joint_msg);
}

void arm::ik_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = joystickMsg->header.stamp;

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

void arm::declareParameters() {
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
  this->declare_parameter("disable_button", 9);
}
void arm::loadParameters() {
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
  this->get_parameter("disable_button", kDisableButton);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}