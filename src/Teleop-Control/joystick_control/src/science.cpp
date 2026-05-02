#include "science.hpp"

science::science() : Node("science_node") {
  declare_parameters();
  load_parameters();
  current_status_ = "-100 Default";
  drill_pub_ = this->create_publisher<std_msgs::msg::Float32>("drill/set", 10);
  elevator_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("elevator/set", 10);
  status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "science/status", 10,
      std::bind(&science::register_status, this, std::placeholders::_1));
  device_control_pub_ = this->create_publisher<sensor_msgs::msg::NavSatStatus>(
      "science/device_control", 10);
  const auto config_timeout = std::chrono::seconds(1);

  for (int i = 0; i < kServoCount; i++) {
    auto last_config_time = std::chrono::steady_clock::now();
    device_control_pub_->publish([this, i]() {
      auto msg = sensor_msgs::msg::NavSatStatus();
      msg.status = -kServoPins[i];
      msg.service = 1 << 14 | kServoFrequencies[i];
      return msg;
    }());
    RCLCPP_INFO(this->get_logger(),
                "Configuring servo %d: button=%d, pin=%d, encoded_value=%u", i,
                kServoButtons[i], kServoPins[i], kServoCommandsEncoded[i]);
    while (std::stoi(current_status_) != 0 &&
           std::stoi(current_status_) != -1) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_config_time > config_timeout) {
        device_control_pub_->publish([this, i]() {
          auto msg = sensor_msgs::msg::NavSatStatus();
          msg.status = -kServoPins[i];
          msg.service = 1 << 14 | kServoFrequencies[i];
          return msg;
        }());
        RCLCPP_INFO(this->get_logger(),
                    "Resending config for servo %d (timeout after 1s)", i);
        last_config_time = now;
      }
      rclcpp::spin_some(this->get_node_base_interface());
    }
    if (std::stoi(current_status_) == 0) {
      RCLCPP_INFO(this->get_logger(), "Servo %d is ready", i);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo %d", i);
    }
    current_status_ = "-100 Default";
  }

  for (int i = 0; i < 4; i++) {
    auto last_config_time = std::chrono::steady_clock::now();
    device_control_pub_->publish([this, i]() {
      auto msg = sensor_msgs::msg::NavSatStatus();
      msg.status = -kSensorPins[i];
      msg.service = kSensorConfigValues[i];
      return msg;
    }());
    RCLCPP_INFO(this->get_logger(),
                "Configuring sensor %d: pin=%d, config_value=%d", i,
                kSensorPins[i], kSensorConfigValues[i]);
    while (std::stoi(current_status_) != 0 &&
           std::stoi(current_status_) != -1) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_config_time > config_timeout) {
        device_control_pub_->publish([this, i]() {
          auto msg = sensor_msgs::msg::NavSatStatus();
          msg.status = -kSensorPins[i];
          msg.service = kSensorConfigValues[i];
          return msg;
        }());
        RCLCPP_INFO(this->get_logger(),
                    "Resending config for sensor %d (timeout after 1s)", i);
        last_config_time = now;
      }
      rclcpp::spin_some(this->get_node_base_interface());
    }
    if (std::stoi(current_status_) == 0) {
      RCLCPP_INFO(this->get_logger(), "Sensor %d is ready", i);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor %d", i);
    }
    current_status_ = "-100 Default";
  }

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&science::science_control, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Science controller started");
};

void science::register_status(
    std::shared_ptr<std_msgs::msg::String> status_msg) {
  current_status_ = status_msg->data;
  RCLCPP_INFO(this->get_logger(), "Received status: %s",
              current_status_.c_str());
}

void science::science_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto drill_msg_ = std_msgs::msg::Float32();
  auto elevator_msg_ = std_msgs::msg::Float32();
  drill_msg_.data = (joystickMsg->axes[kDrillPowerAxis] + 1.0) / 2.0;
  elevator_msg_.data = -(joystickMsg->axes[kDrillElevationAxis]);

  for (int i = 0; i < kServoCount; i++) {
    if (joystickMsg->buttons[kServoButtons[i]]) {
      auto msg = sensor_msgs::msg::NavSatStatus();
      msg.status = kServoPins[i];
      msg.service = kServoCommandsEncoded[i];
      device_control_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Activating servo %d", i);
    }
  }

  drill_pub_->publish(drill_msg_);
  elevator_pub_->publish(elevator_msg_);
};

void science::declare_parameters() {
  this->declare_parameter("drill_elevation_axis", 0);
  this->declare_parameter("drill_power_axis", 0);
  this->declare_parameter("servo_control_axis", 0);
  this->declare_parameter("servo_count", kServoCount);
  for (int i = 0; i < kServoCount; i++) {
    this->declare_parameter("servos.servo" + std::to_string(i) + ".button", 0);
    this->declare_parameter("servos.servo" + std::to_string(i) + ".pin", 0);
    this->declare_parameter("servos.servo" + std::to_string(i) + ".duration",
                            0);
    this->declare_parameter("servos.servo" + std::to_string(i) + ".duty_cycle",
                            0.5);
    this->declare_parameter("servos.servo" + std::to_string(i) + ".frequency",
                            50);
  }
  for (int i = 0; i < 4; i++) {
    this->declare_parameter("sensors.sensor" + std::to_string(i) + ".pin", 0);
    this->declare_parameter("sensors.sensor" + std::to_string(i) + ".type",
                            ANALOG_SENSOR);
    this->declare_parameter("sensors.sensor" + std::to_string(i) + ".name",
                            std::to_string(i) + "sensor");
  }
}

void science::load_parameters() {
  this->get_parameter("drill_power_axis", kDrillPowerAxis);
  this->get_parameter("drill_elevation_axis", kDrillElevationAxis);
  this->get_parameter("servos.servo_count", kServoCount);
  for (int i = 0; i < kServoCount; i++) {
    int button, pin, duration, frequency;
    double value;
    uint16_t encoded_value;
    std::string name;
    this->get_parameter("servos.servo" + std::to_string(i) + ".button", button);
    this->get_parameter("servos.servo" + std::to_string(i) + ".pin", pin);
    this->get_parameter("servos.servo" + std::to_string(i) + ".duration",
                        duration);
    this->get_parameter("servos.servo" + std::to_string(i) + ".duty_cycle",
                        value);
    this->get_parameter("servos.servo" + std::to_string(i) + ".frequency",
                        frequency);
    encoded_value = duration << 10 | (uint16_t)((1 << 10) * value);
    kServoButtons.push_back(button);
    kServoPins.push_back(pin);
    kServoCommandsEncoded.push_back(encoded_value);
    kServoFrequencies.push_back(frequency);
  }
  for (int i = 0; i < 4; i++) {
    int pin, type;
    uint16_t config_value;
    std::string name;
    this->get_parameter("sensors.sensor" + std::to_string(i) + ".pin", pin);
    this->get_parameter("sensors.sensor" + std::to_string(i) + ".type", type);
    this->get_parameter("sensors.sensor" + std::to_string(i) + ".name", name);
    char first_letter = std::tolower(name[0]);
    config_value =
        1 << 15 | (type == ANALOG_SENSOR ? 0 : 1) << 14 | first_letter;
    kSensorPins.push_back(pin);
    kSensorConfigValues.push_back(config_value);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<science>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}