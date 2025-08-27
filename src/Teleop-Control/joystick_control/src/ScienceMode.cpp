#include "ScienceMode.hpp"

ScienceMode::ScienceMode(rclcpp::Node* node) : Mode("Science", node) {
  RCLCPP_INFO(node_->get_logger(), "Science Mode");
  loadParameters();
  platform_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/platform/set", 10);
  drill_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/drill/set", 10);
  servo_client_ = node_->create_client<interfaces::srv::MoveServo>(
      "/science_servo_service");
  led_client_ =
      node_->create_client<std_srvs::srv::SetBool>("/microscope_light");
}

void ScienceMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handlePlatform(joystickMsg);
  handleDrill(joystickMsg);
  handleMicroscope(joystickMsg);
  handleSoilCollection(joystickMsg);
}

void ScienceMode::handlePlatform(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Process input and output linear component
  double value = joystickMsg->axes[kPlatformAxis];
  ros_phoenix::msg::MotorControl platform_control;
  platform_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  platform_control.value = value;
  platform_pub_->publish(platform_control);
}

void ScienceMode::handleDrill(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Turn it on and off
  double drill_value =
      joystickMsg->buttons[kDrillButton] * joystickMsg->axes[kThrottleAxis];
  ros_phoenix::msg::MotorControl drill_control;
  drill_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  drill_control.value = drill_value;
  drill_pub_->publish(drill_control);
}

void ScienceMode::handleMicroscope(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Process input and output linear component
  static double position;
  double value = joystickMsg->axes[kMicroscopeAxis];
  if (value != 0) {
    position += value;
    setServoPosition(kMicroscopeServo, position);
  }
  if (joystickMsg->buttons[kMicroscopeLightButton]) {
    toggleLights();
  }
}

void ScienceMode::handleSoilCollection(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  if (joystickMsg->buttons[kCollectionButton]) {
    setServoPosition(kCollectionServo, kCollectionOpen);
  } else if (joystickMsg->buttons[kCancelCollectionButton]) {
    setServoPosition(kCollectionServo, kCollectionClose);
  } else if (joystickMsg->buttons[kSoilTestButton]) {
    setServoPosition(kCollectionServo, kCollectionSample);
  } else if (joystickMsg->buttons[kSoilLockButton]) {
    setServoPosition(kCollectionServo, kCollectionLock);
  }
}

void ScienceMode::setServoPosition(int port, int position) const {
  auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
  request->port = port;
  request->pos = position;

  // Wait for the service to be available
  if (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available after waiting");
    return;
  }

  servo_client_->async_send_request(request);
}
void ScienceMode::toggleLights() const {
  static bool light_on = false;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !light_on;
  light_on = !light_on;
  if (!led_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available after waiting");
    return;
  }
  led_client_->async_send_request(request);
}

void ScienceMode::declareParameters(rclcpp::Node* node) {
  node->declare_parameter("science_mode.platform_axis", 1);
  node->declare_parameter("science_mode.drill_button", 2);
  node->declare_parameter("science_mode.microscope_axis", 3);
  node->declare_parameter("science_mode.throttle_axis", 3);
  node->declare_parameter("science_mode.soil_collection_button", 4);
  node->declare_parameter("science_mode.cancel_collection_button", 5);
  node->declare_parameter("science_mode.soil_test_button", 6);
  node->declare_parameter("science_mode.microscope_light_button", 7);
  node->declare_parameter("science_mode.collection_servo", 0);
  node->declare_parameter("science_mode.microscope_servo", 1);
  node->declare_parameter("science_mode.collection_open", 0);
  node->declare_parameter("science_mode.collection_dump", 90);
  node->declare_parameter("science_mode.collection_lock", 180);
  node->declare_parameter("science_mode.collection_test", 180);
}

void ScienceMode::loadParameters() {
  node_->get_parameter("science_mode.platform_axis", kPlatformAxis);
  node_->get_parameter("science_mode.drill_button", kDrillButton);
  node_->get_parameter("science_mode.microscope_axis", kMicroscopeAxis);
  node_->get_parameter("science_mode.throttle_axis", kThrottleAxis);
  node_->get_parameter("science_mode.soil_collection_button",
                       kCollectionButton);
  node_->get_parameter("science_mode.cancel_collection_button",
                       kCancelCollectionButton);
  node_->get_parameter("science_mode.soil_test_button", kSoilTestButton);
  node_->get_parameter("science_mode.lock_sample_button", kSoilLockButton);
  node_->get_parameter("science_mode.microscope_light_button",
                       kMicroscopeLightButton);
  node_->get_parameter("science_mode.collection_servo", kCollectionServo);
  node_->get_parameter("science_mode.microscope_servo", kMicroscopeServo);
  node_->get_parameter("science_mode.collection_open", kCollectionOpen);
  node_->get_parameter("science_mode.collection_dump", kCollectionClose);
  node_->get_parameter("science_mode.collection_test", kCollectionSample);
  node_->get_parameter("science_mode.collection_lock", kCollectionLock);
}
