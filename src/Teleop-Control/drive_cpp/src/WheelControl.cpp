#include "WheelControl.hpp"

#include <iostream>

WheelControl::WheelControl(std::string wheel_name, rclcpp::Node* node)
    : node_(node), name_(wheel_name) {
  if (wheel_name.find("Left") != std::string::npos ||
      wheel_name.find("left") != std::string::npos) {
    side_ = WheelSide::LEFT;
  } else if (wheel_name.find("Right") != std::string::npos ||
             wheel_name.find("right") != std::string::npos) {
    side_ = WheelSide::RIGHT;
  } else {
    RCLCPP_FATAL(node->get_logger(),
                 "Wheel name must contain either 'Left' or 'Right'");
    throw std::invalid_argument(
        "Wheel name must contain either 'Left' or 'Right'");
  }
  pub_ = node->create_publisher<MotorControl>(wheel_name + "/set", 10);
  control_.mode = MotorControl::VELOCITY;
}

void WheelControl::setVelocity(double value) { control_.value = value; }

void WheelControl::send() const { pub_->publish(control_); }

void WheelControl::setStatus(const MotorStatus::SharedPtr msg) {
  status_.velocity = msg->velocity;
  status_.temperature = msg->temperature;
  status_.current = msg->output_current;
  status_.voltageIn = msg->bus_voltage;
  status_.voltageOut = msg->output_voltage;
}
