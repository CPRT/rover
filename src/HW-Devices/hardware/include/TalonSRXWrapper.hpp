#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
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

class TalonSRXWrapper {
 public:
  // Constructor
  TalonSRXWrapper(const hardware_interface::ComponentInfo &joint,
                  std::shared_ptr<rclcpp::Node> debug_node);

  void pub_status() const;

  // Updates the TalonSRX motor controller with the given command
  void write();

  // Returns the current position of the TalonSRX motor controller
  void read();

  void add_state_interface(
      std::vector<hardware_interface::StateInterface> &state_interfaces);

  void add_command_interface(
      std::vector<hardware_interface::CommandInterface> &command_interfaces);
  void configure();

  static void setup();

 private:
  const hardware_interface::ComponentInfo info_;

  // Parameters
  int id_;
  double kP_;
  double kI_;
  double kD_;
  double kF_;
  ctre::phoenix::motorcontrol::ControlMode control_type_;
  ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice sensor_type_;
  int sensor_ticks_;
  double sensor_offset_;
  bool crossover_mode_;
  bool inverted_;
  bool invert_sensor_;

  // Interfaces
  double position_;
  double velocity_;
  double command_;

  // Ros2 node and debug publisher
  std::shared_ptr<rclcpp::Node> debug_node_;
  rclcpp::Publisher<ros_phoenix::msg::MotorStatus>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> talon_controller_;
};
