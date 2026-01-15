#include "ScienceMode.hpp"

ScienceMode::ScienceMode(rclcpp::Node *node) : Mode("Science", node) {
  RCLCPP_INFO(node_->get_logger(), "Science Mode");
  loadParameters();

  platform_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/platform/set", 10);
  drill_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/drill/set", 10);

  std::vector<std::string> motor_names = {collectionServo, microscopeServo};

  for (const auto &topic : motor_names) {
    std::string topic_name = "/" + topic;
    motor_pubs[topic] = node_->create_publisher<std_msgs::msg::Float32>(
        topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  }
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
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  // Process input and output linear component
  static double position;
  double value = joystickMsg->axes[kMicroscopeAxis] * rad_multiplier;
  if (value != 0) {
    position += value;
    setServoPosition(microscopeServo, position);
  }
  if (joystickMsg->buttons[kMicroscopeLightButton]) {
    toggleLights();
  }
}

void ScienceMode::handleSoilCollection(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (joystickMsg->buttons[kCollectionButton]) {
    RCLCPP_INFO(node_->get_logger(), "Collection");
    setServoPosition(collectionServo, kCollectionOpen);
  } else if (joystickMsg->buttons[kCancelCollectionButton]) {
    RCLCPP_INFO(node_->get_logger(), "Cancel collection");
    setServoPosition(collectionServo, kCollectionClose);
  } else if (joystickMsg->buttons[kSoilTestButton]) {
    RCLCPP_INFO(node_->get_logger(), "Soil test");
    setServoPosition(collectionServo, kCollectionSample);
  } else if (joystickMsg->buttons[kSoilLockButton]) {
    RCLCPP_INFO(node_->get_logger(), "Soil lock");
    setServoPosition(collectionServo, kCollectionLock);
  }
}

void ScienceMode::setServoPosition(std::string name, double position) {
  auto servo_msg = std_msgs::msg::Float32();
  servo_msg.data = position;
  motor_pubs[name]->publish(servo_msg);
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

void ScienceMode::declareParameters(rclcpp::Node *node) {
  node->declare_parameter("science_mode.platform_axis", 1);
  node->declare_parameter("science_mode.drill_button", 2);
  node->declare_parameter("science_mode.microscope_axis", 3);
  node->declare_parameter("science_mode.throttle_axis", 3);
  node->declare_parameter("science_mode.soil_collection_button", 4);
  node->declare_parameter("science_mode.cancel_collection_button", 5);
  node->declare_parameter("science_mode.soil_test_button", 6);
  node->declare_parameter("science_mode.microscope_light_button", 7);
  node->declare_parameter("science_mode.collection_servo_name", "collection");
  node->declare_parameter("science_mode.microscope_servo_name", "microscope");
  node->declare_parameter("science_mode.collection_open", 0.0);
  node->declare_parameter("science_mode.collection_dump", M_PI / 2);
  node->declare_parameter("science_mode.collection_lock", M_PI);
  node->declare_parameter("science_mode.collection_test", M_PI);
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
  node_->get_parameter("science_mode.collection_servo_name", collectionServo);
  node_->get_parameter("science_mode.microscope_servo_name", microscopeServo);
  node_->get_parameter("science_mode.collection_open", kCollectionOpen);
  node_->get_parameter("science_mode.collection_dump", kCollectionClose);
  node_->get_parameter("science_mode.collection_test", kCollectionSample);
  node_->get_parameter("science_mode.collection_lock", kCollectionLock);
}
