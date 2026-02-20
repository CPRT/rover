#include "science.hpp"

science::science() : Node("science_node") {
  declare_parameters();
  load_parameters();
  drill_pub_ = this->create_publisher<std_msgs::msg::Float32>("drill/set", 10);
  elevator_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("elevator/set", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&science::science_control, this, std::placeholders::_1));

  for (int i = 0; i < kServoNames.size(); i++) {
    servo_values_.push_back(kServoDefaultValues[i]);
    servo_pubs_.push_back(
        this->create_publisher<std_msgs::msg::UInt32>(kServoNames[i], 10));
  }

  RCLCPP_INFO(this->get_logger(), "Science controller started");
};

void science::science_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto drill_msg_ = std_msgs::msg::Float32();
  auto elevator_msg_ = std_msgs::msg::Float32();
  if (joystickMsg->buttons[kDrillButton]) {
    drill_msg_.data = 1.0;
  } else {
    drill_msg_.data = 0.0;
  }
  elevator_msg_.data = -(joystickMsg->axes[kDrillElevationAxis]);

  for (int i = 0; i < kServoNames.size(); i++) {
    update_servo_value(i, *joystickMsg);
  }

  drill_pub_->publish(drill_msg_);
  elevator_pub_->publish(elevator_msg_);
};

void science::update_servo_value(int servo_index,
                                 const sensor_msgs::msg::Joy &joystickMsg) {
  if (servo_index < 0 || servo_index >= kServoButtons.size()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid servo index: %d", servo_index);
    return;
  }

  int button = kServoButtons[servo_index];
  int max_value = kServoMaxValues[servo_index];
  int min_value = kServoMinValues[servo_index];

  if (servo_values_[servo_index] <= max_value &&
      servo_values_[servo_index] >= min_value) {
    if (joystickMsg.buttons[button]) {
      if (joystickMsg.axes[kServoControlAxis] == 1.0) {
        if (servo_values_[servo_index] + (max_value - min_value) / 10 >
            max_value) {
          servo_values_[servo_index] = max_value;
        } else {
          servo_values_[servo_index] += (max_value - min_value) / 10;
        }
        servo_pubs_[servo_index]->publish(
            std_msgs::msg::UInt32().set__data(servo_values_[servo_index]));
      } else if (joystickMsg.axes[kServoControlAxis] == -1.0) {
        if (servo_values_[servo_index] - (max_value - min_value) / 10 <
            min_value) {
          servo_values_[servo_index] = min_value;
        } else {
          servo_values_[servo_index] -= (max_value - min_value) / 10;
        }
        servo_pubs_[servo_index]->publish(
            std_msgs::msg::UInt32().set__data(servo_values_[servo_index]));
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error: %s value out of bounds: %d",
                 kServoNames[servo_index].c_str(), servo_values_[servo_index]);
    servo_values_[servo_index] = min_value;
    servo_pubs_[servo_index]->publish(
        std_msgs::msg::UInt32().set__data(servo_values_[servo_index]));
  }
}

void science::declare_parameters() {
  this->declare_parameter("drill_button", 1);
  this->declare_parameter("drill_elevation_axis", 2);
  this->declare_parameter("servo_control_axis", 5);

  this->declare_parameter("servo1.button", 12);
  this->declare_parameter("servo1.max_value", 65535);
  this->declare_parameter("servo1.min_value", 0);
  this->declare_parameter("servo1.default_value", 0);
  this->declare_parameter("servo1.name", "servo1");

  this->declare_parameter("servo2.button", 13);
  this->declare_parameter("servo2.max_value", 65535);
  this->declare_parameter("servo2.min_value", 0);
  this->declare_parameter("servo2.default_value", 0);
  this->declare_parameter("servo2.name", "servo2");

  this->declare_parameter("servo3.button", 14);
  this->declare_parameter("servo3.max_value", 65535);
  this->declare_parameter("servo3.min_value", 0);
  this->declare_parameter("servo3.default_value", 0);
  this->declare_parameter("servo3.name", "servo3");

  this->declare_parameter("servo4.button", 15);
  this->declare_parameter("servo4.max_value", 65535);
  this->declare_parameter("servo4.min_value", 0);
  this->declare_parameter("servo4.default_value", 0);
  this->declare_parameter("servo4.name", "servo4");

  this->declare_parameter("servo5.button", 16);
  this->declare_parameter("servo5.max_value", 65535);
  this->declare_parameter("servo5.min_value", 0);
  this->declare_parameter("servo5.default_value", 0);
  this->declare_parameter("servo5.name", "servo5");

  this->declare_parameter("servo6.button", 17);
  this->declare_parameter("servo6.max_value", 65535);
  this->declare_parameter("servo6.min_value", 0);
  this->declare_parameter("servo6.default_value", 0);
  this->declare_parameter("servo6.name", "servo6");

  this->declare_parameter("servo7.button", 2);
  this->declare_parameter("servo7.max_value", 65535);
  this->declare_parameter("servo7.min_value", 0);
  this->declare_parameter("servo7.default_value", 0);
  this->declare_parameter("servo7.name", "servo7");

  this->declare_parameter("servo8.button", 1);
  this->declare_parameter("servo8.max_value", 65535);
  this->declare_parameter("servo8.min_value", 0);
  this->declare_parameter("servo8.default_value", 0);
  this->declare_parameter("servo8.name", "servo8");
}

void science::load_parameters() {
  std::string servo_name;
  int servo_button;
  int servo_max_value;
  int servo_min_value;
  int servo_default_value;

  this->get_parameter("drill_button", kDrillButton);
  this->get_parameter("drill_elevation_axis", kDrillElevationAxis);
  this->get_parameter("servo_control_axis", kServoControlAxis);
  this->get_parameter("servo1.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo1.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo1.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo1.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo1.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);

  this->get_parameter("servo2.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo2.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo2.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo2.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo2.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);

  this->get_parameter("servo3.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo3.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo3.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo3.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo3.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);

  this->get_parameter("servo4.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo4.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo4.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo4.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo4.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);

  this->get_parameter("servo5.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo5.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo5.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo5.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo5.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);

  this->get_parameter("servo6.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo6.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo6.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo6.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo6.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);

  this->get_parameter("servo7.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo7.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo7.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo7.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo7.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);

  this->get_parameter("servo8.name", servo_name);
  kServoNames.push_back(servo_name);
  this->get_parameter("servo8.button", servo_button);
  kServoButtons.push_back(servo_button);
  this->get_parameter("servo8.max_value", servo_max_value);
  kServoMaxValues.push_back(servo_max_value);
  this->get_parameter("servo8.min_value", servo_min_value);
  kServoMinValues.push_back(servo_min_value);
  this->get_parameter("servo8.default_value", servo_default_value);
  kServoDefaultValues.push_back(servo_default_value);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<science>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}