#include "WheelControl.hpp"

#include <iostream>

WheelControl::WheelControl(std::string wheel_name, rclcpp::Node *node,
                           bool is_open_loop, double open_loop_scalar)
    : node_(node), name_(wheel_name), is_open_loop_(is_open_loop),
      open_loop_scalar_(open_loop_scalar) {
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

  if (is_open_loop_) {
    control_.mode = MotorControl::PERCENT_OUTPUT;
    RCLCPP_INFO(node->get_logger(),
                "Wheel %s configured for open loop control with scalar %.3f",
                wheel_name.c_str(), open_loop_scalar_);
  } else {
    control_.mode = MotorControl::VELOCITY;
    RCLCPP_INFO(node->get_logger(),
                "Wheel %s configured for closed loop velocity control",
                wheel_name.c_str());
  }
}

void WheelControl::setVelocity(double value) {
  if (is_open_loop_) {
    control_.value = value * open_loop_scalar_;
  } else {
    control_.value = value;
  }
}

void WheelControl::send() const { pub_->publish(control_); }

void WheelControl::setStatus(const MotorStatus::SharedPtr msg) {
  status_.velocity = msg->velocity;
  status_.temperature = msg->temperature;
  status_.current = msg->output_current;
  status_.voltageIn = msg->bus_voltage;
  status_.voltageOut = msg->output_voltage;
}
