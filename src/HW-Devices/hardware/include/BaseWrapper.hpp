#pragma once

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class BaseWrapper {
public:
  using StateInterfaceVec = std::vector<hardware_interface::StateInterface>;
  using CommandInterfaceVec = std::vector<hardware_interface::CommandInterface>;

  virtual ~BaseWrapper() = default;

  /// Publish any debug/telemetry status for this motor controller
  virtual void pub_status() const = 0;

  /// Push current command to hardware (e.g. percent output, position, velocity)
  virtual void write() = 0;

  /// Read current state from hardware (e.g. position, velocity)
  virtual void read() = 0;

  /// Register state interfaces for this joint with ROS 2 control
  virtual void add_state_interface(StateInterfaceVec &state_interfaces) = 0;

  /// Register command interfaces for this joint with ROS 2 control
  virtual void
  add_command_interface(CommandInterfaceVec &command_interfaces) = 0;

  /// Configure from `hardware_interface::ComponentInfo` and ROS parameters
  virtual void configure();

  /// Optional: identify controller type or name
  virtual std::string get_name() const = 0;

  /// Optional: CAN / device ID or similar identifier
  virtual int get_id() const = 0;

protected:
  BaseWrapper(const hardware_interface::ComponentInfo &joint,
              std::shared_ptr<rclcpp::Node> debug_node);

  // Common configuration info from ros2_control
  const hardware_interface::ComponentInfo info_;

  // Common interfaces
  double position_;
  double velocity_;
  double command_;

  // ROS 2 node and debug publisher/timer (usable by all derived controllers)
  std::shared_ptr<rclcpp::Node> debug_node_;
  int freq_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};
