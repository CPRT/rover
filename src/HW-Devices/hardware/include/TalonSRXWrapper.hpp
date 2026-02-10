#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

#include "BaseWrapper.hpp"

class TalonSRXWrapper : public BaseWrapper {
public:
  TalonSRXWrapper(const hardware_interface::ComponentInfo &joint,
                  rclcpp::Node::SharedPtr debug_node);

  void pub_status() const override;

  void write() override;

  void read() override;

  void configure() override;

  // Optional: type-level setup for Talons if you need it
  static void setup();

private:
  enum class SensorType { PWM, RELATIVE, ANALOG };

  // Parameters
  int id_;
  double kP_;
  double kI_;
  double kD_;
  double kF_;
  ctre::phoenix::motorcontrol::ControlMode control_type_;
  SensorType sensor_type_;
  int sensor_ticks_;
  double sensor_offset_;
  bool crossover_mode_;
  bool inverted_;
  bool invert_sensor_;
  rclcpp::Publisher<ros_phoenix::msg::MotorStatus>::SharedPtr debug_pub_;

  // Talon-specific handle
  std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> talon_controller_;
};
