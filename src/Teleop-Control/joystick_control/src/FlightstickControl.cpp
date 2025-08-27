#include "FlightstickControl.hpp"

FlightstickControl::FlightstickControl()
    : Node("flightstick_control"),
      mode_(nullptr),
      currentMode_(ModeType::NONE) {
  declareParameters();
  loadParameters();
  changeMode(currentMode_);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&FlightstickControl::processJoystick, this,
                std::placeholders::_1));

  status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/rover_mode", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  light_pub_ = this->create_publisher<std_msgs::msg::Int8>("/light", qos);
}

void FlightstickControl::processJoystick(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (checkForModeChange(joystickMsg)) {
    return;
  }
  if (mode_ != nullptr && currentMode_ != ModeType::NONE) {
    mode_->processJoystickInput(joystickMsg);
  }
}

bool FlightstickControl::checkForModeChange(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  static const std::map<int, ModeType> buttonToMode = {
      {kDriveModeButton, ModeType::DRIVE},
      {kArmIKModeButton, ModeType::ARM_IK},
      {kArmManualModeButton, ModeType::ARM_MANUAL},
      {kArmDummyButton, ModeType::ARM_DUMMY},
      {kNavModeButton, ModeType::NAV},
      {kScienceModeButton, ModeType::SCIENCE}};
  for (const auto& [buttonIndex, mode] : buttonToMode) {
    if (joystickMsg->buttons[buttonIndex]) {
      if (!checkAxes(joystickMsg, mode)) {
        RCLCPP_WARN(this->get_logger(),
                    "At least one axis is not 0, can't switch modes");
        return false;
      }
      return changeMode(mode);
    }
  }

  return false;
}

bool FlightstickControl::checkAxes(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg, ModeType nextMode) {
  // key is mode, vector is axes
  static const std::map<ModeType, std::vector<int>> modeParameters = {
      {ModeType::DRIVE, {0, 1, 3, 4}},
      {ModeType::ARM_IK, {0, 1, 4, 5, 6}},
      {ModeType::ARM_MANUAL, {0, 1, 4, 5, 6, 7}},
      {ModeType::SCIENCE, {1, 3}},
      {ModeType::ARM_DUMMY, {0, 1, 4, 5, 6, 7}}};  // add nav?
  auto it = modeParameters.find(nextMode);
  if (it == modeParameters.end()) {
    return true;
  }
  const auto& axesIndices = it->second;
  for (const int& index : axesIndices) {
    if (index >= joystickMsg->axes.size()) {
      continue;
    }
    if (joystickMsg->axes[index] > 0.01) {
      return false;
    }
  }
  return true;
}
bool FlightstickControl::changeMode(ModeType mode) {
  if (currentMode_ == mode) {
    return false;
  }
  currentMode_ = mode;
  auto message = std_msgs::msg::String();
  switch (mode) {
    case ModeType::NONE:
      mode_ = nullptr;
      message.data = "Idle";
      status_pub_->publish(message);
      break;
    case ModeType::DRIVE:
      RCLCPP_INFO(this->get_logger(), "Entering Drive Mode");
      mode_ = std::make_unique<DriveMode>(this);
      message.data = "Drive";
      status_pub_->publish(message);
      break;
    case ModeType::ARM_MANUAL:
      RCLCPP_INFO(this->get_logger(), "Entering Manual Mode");
      mode_ = std::make_unique<ArmManualMode>(this);
      message.data = "Manual";
      status_pub_->publish(message);
      break;
    case ModeType::SCIENCE:
      RCLCPP_INFO(this->get_logger(), "Entering Science Mode");
      mode_ = std::make_unique<ScienceMode>(this);
      message.data = "Science";
      status_pub_->publish(message);
      break;
    case ModeType::ARM_IK:
      RCLCPP_INFO(this->get_logger(), "Entering IK Mode");
      mode_ = std::make_unique<ArmIKMode>(this);
      message.data = "IK";
      status_pub_->publish(message);
      break;
    case ModeType::ARM_DUMMY:
      RCLCPP_INFO(this->get_logger(), "Entering Arm Dumb Mode");
      mode_ = std::make_unique<ArmDummyMode>(this);
      message.data = "Dummy";
      status_pub_->publish(message);
      break;
    default:
      RCLCPP_WARN(this->get_logger(),
                  "Mode not implemented, returning to NONE");
      currentMode_ = ModeType::NONE;
      mode_ = nullptr;
      message.data = "Idle";
      status_pub_->publish(message);
      return false;
  }
  auto msg = std_msgs::msg::Int8();
  msg.data = kTeleopLightMode;
  light_pub_->publish(msg);
  return true;
}

void FlightstickControl::declareParameters() {
  this->declare_parameter("drive_mode_button", 12);
  this->declare_parameter("arm_ik_mode_button", 11);
  this->declare_parameter("arm_manual_mode_button", 10);
  this->declare_parameter("nav_mode_button", 13);
  this->declare_parameter("science_mode_button", 14);
  this->declare_parameter("arm_dummy_mode_button", 8);
  this->declare_parameter("teleop_light_mode", 1);
  DriveMode::declareParameters(this);
  ArmManualMode::declareParameters(this);
  ArmIKMode::declareParameters(this);
  ScienceMode::declareParameters(this);
}

void FlightstickControl::loadParameters() {
  this->get_parameter("drive_mode_button", kDriveModeButton);
  this->get_parameter("arm_ik_mode_button", kArmIKModeButton);
  this->get_parameter("arm_manual_mode_button", kArmManualModeButton);
  this->get_parameter("nav_mode_button", kNavModeButton);
  this->get_parameter("science_mode_button", kScienceModeButton);
  this->get_parameter("arm_dummy_mode_button", kArmDummyButton);
  this->get_parameter("teleop_light_mode", kTeleopLightMode);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightstickControl>());
  rclcpp::shutdown();
  return 0;
}