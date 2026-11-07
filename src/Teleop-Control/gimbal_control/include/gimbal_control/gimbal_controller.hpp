#pragma once
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/bool.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>

namespace gimbal_control {
class GimbalController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override;

private:
  struct Command {
    std::array<double, 3> velocity{};
    std::chrono::steady_clock::time_point received{};
    uint64_t sequence{0};
  };
  std::vector<std::string> joints_;
  std::vector<double> lower_, upper_, max_velocity_;
  std::array<double, 3> positions_{};
  std::array<size_t, 3> command_indices_{}, state_indices_{};
  double timeout_{0.5};
  bool homing_{false};
  uint64_t ignored_sequence_{0};
  std::atomic<uint64_t> sequence_{0};
  std::atomic<bool> active_{false}, reset_held_{false}, reset_pending_{false};
  realtime_tools::RealtimeBuffer<Command> command_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
};
} // namespace gimbal_control
