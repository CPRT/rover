#pragma once

#include <functional>
#include <optional>
#include <string>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

class Axle {
public:
  Axle() = delete;
  Axle(const std::string &steering_angle_name,
       const std::string &wheel_velocity_name)
      : steering_angle_name_(steering_angle_name),
        wheel_velocity_name_(wheel_velocity_name) {}

  bool activate(
      std::vector<hardware_interface::LoanedCommandInterface>
          &command_interfaces,
      std::vector<hardware_interface::LoanedStateInterface> &state_interfaces);

  // Write commands into hardware interfaces (if assigned)
  bool update(double steering_angle_cmd, double wheel_velocity_cmd);
  bool stop();

  // Read current state (if state interfaces are assigned)
  std::optional<double> currentAngle() const;
  std::optional<double> currentVelocity() const;

  std::string steering_angle_name() const { return steering_angle_name_; }
  std::string wheel_velocity_name() const { return wheel_velocity_name_; }

private:
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      steering_angle_;
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      wheel_velocity_;
  std::optional<
      std::reference_wrapper<const hardware_interface::LoanedStateInterface>>
      steering_angle_state_;
  std::optional<
      std::reference_wrapper<const hardware_interface::LoanedStateInterface>>
      wheel_velocity_state_;
  const std::string steering_angle_name_;
  const std::string wheel_velocity_name_;
};
