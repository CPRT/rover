#include "gimbal_control/gimbal_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>

namespace gimbal_control {
using CallbackReturn = controller_interface::CallbackReturn;
CallbackReturn GimbalController::on_init() {
  auto_declare<std::vector<std::string>>(
      "joints", {"gimbal_pan_joint", "gimbal_tilt_joint", "mast_camera_joint"});
  auto_declare<std::vector<double>>("lower_limits", {-3.10, -3.75, -4.2});
  auto_declare<std::vector<double>>("upper_limits", {3.18, 2.53, 2.08});
  auto_declare<std::vector<double>>("max_velocity", {0.5, 0.5, 0.5});
  auto_declare<double>("command_timeout", 0.5);
  return CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration
GimbalController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config{
      controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto &joint : joints_)
    config.names.push_back(joint + "/position");
  return config;
}
controller_interface::InterfaceConfiguration
GimbalController::state_interface_configuration() const {
  return command_interface_configuration();
}
CallbackReturn GimbalController::on_configure(const rclcpp_lifecycle::State &) {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  lower_ = get_node()->get_parameter("lower_limits").as_double_array();
  upper_ = get_node()->get_parameter("upper_limits").as_double_array();
  max_velocity_ = get_node()->get_parameter("max_velocity").as_double_array();
  timeout_ = get_node()->get_parameter("command_timeout").as_double();
  if (joints_.size() != 3 || lower_.size() != 3 || upper_.size() != 3 ||
      max_velocity_.size() != 3 || !std::isfinite(timeout_) || timeout_ <= 0)
    return CallbackReturn::ERROR;
  for (size_t i = 0; i < 3; ++i) {
    if (joints_[i].empty() || !std::isfinite(lower_[i]) ||
        !std::isfinite(upper_[i]) || !std::isfinite(max_velocity_[i]) ||
        lower_[i] >= upper_[i] || lower_[i] > 0 || upper_[i] < 0 ||
        max_velocity_[i] <= 0)
      return CallbackReturn::ERROR;
    for (size_t j = 0; j < i; ++j)
      if (joints_[i] == joints_[j])
        return CallbackReturn::ERROR;
  }
  twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!active_.load())
          return;
        Command command;
        command.velocity = {msg->angular.z, msg->angular.y, msg->angular.x};
        // Invalid input stops all motion, rather than leaving the old velocity
        // active.
        for (double v : command.velocity)
          if (!std::isfinite(v)) {
            command.velocity.fill(0);
            break;
          }
        command.received = std::chrono::steady_clock::now();
        command.sequence = ++sequence_;
        command_.writeFromNonRT(command);
      });
  reset_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/reset", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (!active_.load())
          return;
        reset_held_.store(msg->data);
        if (msg->data)
          reset_pending_.store(true);
      });
  return CallbackReturn::SUCCESS;
}
CallbackReturn GimbalController::on_activate(const rclcpp_lifecycle::State &) {
  if (command_interfaces_.size() != 3 || state_interfaces_.size() != 3)
    return CallbackReturn::ERROR;
  for (size_t i = 0; i < 3; ++i) {
    auto name = joints_[i] + "/position";
    auto c =
        std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                     [&](const auto &v) { return v.get_name() == name; });
    auto s = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                          [&](const auto &v) { return v.get_name() == name; });
    if (c == command_interfaces_.end() || s == state_interfaces_.end())
      return CallbackReturn::ERROR;
    command_indices_[i] = std::distance(command_interfaces_.begin(), c);
    state_indices_[i] = std::distance(state_interfaces_.begin(), s);
    positions_[i] = s->get_value();
    if (!std::isfinite(positions_[i]) || positions_[i] < lower_[i] ||
        positions_[i] > upper_[i])
      return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < 3; ++i)
    command_interfaces_[command_indices_[i]].set_value(positions_[i]);
  ignored_sequence_ = sequence_.load();
  reset_held_ = false;
  reset_pending_ = false;
  homing_ = false;
  active_ = true;
  return CallbackReturn::SUCCESS;
}
CallbackReturn
GimbalController::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  return CallbackReturn::SUCCESS; // Position commands hold the last target.
}
controller_interface::return_type
GimbalController::update(const rclcpp::Time &, const rclcpp::Duration &period) {
  if (!active_)
    return controller_interface::return_type::OK;
  const auto command = *command_.readFromRT();
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0 || dt > timeout_)
    return controller_interface::return_type::OK;
  if (reset_pending_.exchange(false))
    homing_ = true;
  const bool resetting = homing_ || reset_held_.load();
  const double age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - command.received)
                         .count();
  bool centered = true;
  for (size_t i = 0; i < 3; ++i) {
    double delta = 0;
    const double step = max_velocity_[i] * dt;
    if (resetting)
      delta = std::clamp(-positions_[i], -step, step);
    else if (command.sequence > ignored_sequence_ && age >= 0 &&
             age <= timeout_)
      delta =
          std::clamp(command.velocity[i], -max_velocity_[i], max_velocity_[i]) *
          dt;
    positions_[i] = std::clamp(positions_[i] + delta, lower_[i], upper_[i]);
    command_interfaces_[command_indices_[i]].set_value(positions_[i]);
    centered = centered && std::abs(positions_[i]) < 1e-9;
  }
  if (resetting) {
    ignored_sequence_ = command.sequence;
    if (centered)
      homing_ = false;
  }
  return controller_interface::return_type::OK;
}
} // namespace gimbal_control
PLUGINLIB_EXPORT_CLASS(gimbal_control::GimbalController,
                       controller_interface::ControllerInterface)
